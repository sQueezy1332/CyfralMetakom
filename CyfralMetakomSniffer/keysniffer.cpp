#include "keysniffer.h"

Keysniffer::Keysniffer() {
#if defined(__LGT8FX8P__)
	//C0XR |= bit(C0FEN) | bit(C0FS0);  //filter delay 32us
	C0SR = bit(C0BG); //comparator enable, DAC output on positive input
	sbi(ADCSRB, ACME); //CME00, multiplexer adc on inverse input //A0 default
	DACON = bit(DACEN) | bit(DAVS1); //DAC enable, internal reference voltage
	//VCAL = VCAL2; sbi(ADCSRD, REFS1);//2.048v
	VCAL = VCAL3; sbi(ADCSRD, REFS2);//4.096v
	DALR = (compRefVoltage * 256 / refVoltage - 1);
	//sbi(DACON, DAOE); //DAC output on pd4  //debug
#ifdef VOLTAGE_MEASURING
	ADMUX = ((uint8_t)refVoltage << REFS0 /*| 0b0001*/); //ADCSRD |= (0b10 < IVSEL0); //pc1
	ADCSRA = (bit(ADEN) | bit(ADSC) | 0b110); //64 prescaler, 500khz 30us
#endif // VOLTAGE_MEASURING
	//pMode(5, OUTPUT); dWrite(5, 1);
#elif defined(__AVR_ATmega328P__)
	pInit(pin_pullup, OUTPUT); dWrite(pin_pullup, HIGH);
	ADMUX |= bit(REFS0); //AVCC
	ADCSRA = bit(ADEN) | bit(ADSC) | 0b110; //adc enable, 64 prescaler, 250khz, 52us //8mhz 1.6us default
#endif
#if defined(ARDUINO_ARCH_ESP32)
	pInit(pin_pullup, INPUT);
#endif
	pInit(pin_comparator, INPUT);
	pInit(pin_data, INPUT);
}

bool Keysniffer::KeyDetection(byte(&buf)[8]) {
	register byte startNibble, bitmask;
	word startPeriod;
	dword tempVoltage, timer;
	while (!flagInterrupt) {
#ifdef VOLTAGE_MEASURING
		if (flagAdcFirstConv) {//DEBUG(adc()); continue;
			if ((tempVoltage = Kalman()) < adcTriggerValue) {
				wdr;
				for (byte i = 0; i < (0xFF >> (8 - adcAvgof)); i++) {
					tempVoltage += Kalman();
				}
				word result = tempVoltage >> adcAvgof;
				if (result < adcTriggerValue) {
					flagAdcFirstConv = false;
					avgVoltage = result;
				}
				//DEBUG(avgVoltage); continue; digitalWrite(13, 1);
			}
			else continue;
		}
#endif // VOLTAGE_MEASURING
	Start:
		error = NO_ERROR;
		if (comparator()) { continue; } //wait until high signal is start
		timer = uS + dutyLow + 450;		//50 - 230 datasheet ~400 for last bit + synchro
		wdr;
		while (!comparator()) {			//Try read METAKOM synchronise bit log 0
			if (uS > timer) {
				timer = mS + 200;
				while (!comparator()) {
					if (mS > timer) {
						error = ERROR_VERY_LONG_LOW;
						DEBUG(error);
						return false;
					}
				}
				//error = ERROR_SYNC_BIT; DEBUG(error);
				goto Start;
			}
		}
		startPeriod = uS - timer + dutyLow;
		for (bitmask = 0b100, startNibble = 0; bitmask; bitmask >>= 1) {
			if (recvBitMetakom())
				startNibble |= bitmask;
			if (startNibble > METAKOM || error)
				goto Start;
		}
		if (startNibble == METAKOM && startPeriod >= period) {
			if (Metakom(buf)) return METAKOM;
			DEBUG(error);
			continue;
		}//last bit could be volatile if Cyfral
		timer = uS;
		while (comparator()) {				//wait 1 duty cycle for Cyfral 8 half bits start nibble 
			if (uS - timer > 200) {
				//error = ERROR_START_DUTY_HIGH; DEBUG(error);
				goto Start;
			}
		}
		if ((uS - timer) > dutyLow || recvBitCyfral()) {  //duty low from previos read bit Metakom or 0b0_0001
			if (Cyfral(buf))
				return CYFRAL;
			DEBUG(error);
			continue;
		}
		error = ERROR_NOT_RECOGNIZED; DEBUG(error); //continue; //debug
		return false;
	}
	return false;
}

bool Keysniffer::Metakom(byte(&buf)[8]) {
	register byte count1 = 0, count0 = 0, parity = 0, i, bitmask;
	dword startTimer;
	for (i = 0; i < 3; i++) {
		for (bitmask = 128; bitmask; bitmask >>= 1) {
			if (recvBitMetakom()) {
				buf[i] |= bitmask;
				T.t1 += period;
				T.ti1 += dutyHigh;
				count1++;
			}
			else {
				if (error) goto exit;
				T.t0 += period;
				T.ti0 += dutyHigh;
				count0++;
			}
		}
	}
	for (bitmask = 128; bitmask != 1; bitmask >>= 1) {
		if (recvBitMetakom()) {
			buf[i] |= bitmask;
			T.t1 += period;
			T.ti1 += dutyHigh;
			count1++;
			parity++;			//counting log 1 in last byte
		}
		else {
			if (error) goto exit;
			T.t0 += period;
			T.ti0 += dutyHigh;
			count0++;
		}
	}
	startTimer = uS;
	while (comparator()) {
		if (uS - startTimer > 200) {
			error = ERROR_DUTY_HIGH_METAKOM;
			goto exit;
		}
	}
	dutyHigh = uS - startTimer;
	if (parity & 1) {			//check and set parity only for last byte
		buf[i] |= 1;
		T.t1 += period;			// previos period
		T.ti1 += dutyHigh;
		count1++;
	}
	else {
		T.t0 += period;
		T.ti0 += dutyHigh;
		count0++;
	}
	buf[4] = T.t1 / count1;		// division  for Period 1
	buf[6] = T.ti1 / count1;
	if (count0) {				//checking for divider not be zero
		buf[5] = T.t0 / count0;	// division  for Period 0
		buf[7] = T.ti0 / count0;
	}
	clearVars();
	return true;
exit:
	clearVars();
	memset(buf, 0, i);
	return false;
}

bool Keysniffer::recvBitMetakom() {
	dword startTimer = uS;
	while (comparator()) {
		if (uS - startTimer > 200) {
			error = ERROR_DUTY_HIGH_METAKOM;
			return false;
		}
	}
	dutyHigh = uS - startTimer;
	while (!comparator()) {
		if (uS - startTimer - dutyHigh > 160) {
			dutyLow = 160;
			error = ERROR_DUTY_LOW_METAKOM;
			return false;		//may be synchronise bit
		}
	}
	dutyLow = uS - startTimer - dutyHigh;
	if ((period = dutyLow + dutyHigh) < 50) {
		error = ERROR_PERIOD_METAKOM;
		return false;
	}
	return (dutyHigh > dutyLow);
}

bool Keysniffer::Cyfral(byte(&buf)[8]) {
	register byte i = 0, bitmask, nibble;
	do {
		for (nibble = 0, bitmask = 0b1000; bitmask; bitmask >>= 1) {
			if (recvBitCyfral()) {
				nibble |= bitmask;
				T.t1 += period;
				T.ti1 += dutyLow;
			}
			else {
				if (error) goto exit;
				T.t0 += period;
				T.ti0 += dutyLow;
			}
		} //Serial.println(nibble, HEX);
		switch (nibble) {
		case 0x7: case 0xB: case 0xD:case 0xE:
			if (buf[i] & 0xF) {
				buf[i] |= nibble << 4;  //writing nibble from MSB
				break;
				
			}
			else buf[i] |= nibble;
			continue;
		case 0x1:
			memset(buf, 0, i);
			clearVars();
			i = 0;
			continue;
		default:
			error = ERROR_NIBBLE_CYFRAL;
			++i;
			goto exit;
		}
	} while (++i < 4);//fast division by 24 = (65536 / (200 * 32) - 1) * 24 == 221.76; x = (2 ^ 8) / 24 + 1 = 11,66...
	buf[4] = (T.t1 * 11) >> 8;
	buf[6] = (T.ti1 * 11) >> 8;
	buf[5] = T.t0 >> 3;			// division by 8
	buf[7] = T.ti0 >> 3;
	clearVars();
	return true;
exit:
	clearVars();
	return false;
}

bool Keysniffer::recvBitCyfral() {
	dword startTimer = uS;
	while (!comparator()) {
		if (uS - startTimer > 160) {
			error = ERROR_DUTY_LOW_CYFRAL;
			return false;
		}
	}
	dutyLow = uS - startTimer;
	while (comparator()) {
		if (uS - startTimer - dutyLow > 200) {
			error = ERROR_DUTY_HIGH_CYFRAL;
			return false;
		}
	}
	dutyHigh = uS - startTimer - dutyLow;
	if ((period = uS - startTimer) < 50) {
		error = ERROR_PERIOD_CYFRAL;
		return false;
	}
	return (dutyHigh > dutyLow);
}

bool Keysniffer::comparator() {
	static byte iterator = WAIT_RETRY_COUNT;
	static byte old_iterator;
	static char temp;
	for (;;) {
		if (COMP) {
			while (COMP) {
				if (iterator) {
					iterator--;
				}
				else {//dWrite(5, 0);
					return false;
				}
			}
			old_iterator = iterator;
			while (!COMP) {
				if (iterator < WAIT_RETRY_COUNT) {
					iterator++;
				}
				else {//dWrite(5, 1);
					return true;
				}
			}
			temp = (iterator - ((iterator - old_iterator) * 2)) - 1;
			if (temp < 1) {
				iterator = 0;
				return false;
			}
			else iterator = temp;
		}
		else {
			if (iterator < WAIT_RETRY_COUNT) {
				iterator++;
			}
			else {//dWrite(5, 1);
				return true;
			}
		}
	}
}

#if defined VOLTAGE_MEASURING || defined (__AVR__)
word Keysniffer::Kalman() {
	static word old_kalman = adcMaxValue;
	ADCSRA |= bit(ADSC);
	while (ADCSRA & bit(ADSC)) {};
	return old_kalman = (ADC >> 1) + (old_kalman >> 1) + 1;
}
#endif
void Keysniffer::Emulate(const byte buf[], byte keyType, byte emulRetry) {
	if (buf[4] && buf[6]) {
		T.t1 = buf[4];
		T.t0 = buf[5];
		T.ti1 = buf[6];
		T.ti0 = buf[7];
	}
	else {
		if (keyType == CYFRAL) { T.t1 = T.t0 = 180; T.ti1 = 60; T.ti0 = 120; }
		else if (keyType == METAKOM) { T.t1 = T.t0 = 180; T.ti1 = 120; T.ti0 = 60; }
		else return;
	}
	const byte Tj1 = T.t1 - T.ti1;
	const byte Tj0 = T.t0 - T.ti0;
	EMULATE_HIGH_ON(pin_data);
	switch (keyType) {
	case CYFRAL: {
		for (byte retry = 0, bitmask, i; retry < emulRetry; retry++) {
			for (bitmask = 0b1000; bitmask; bitmask >>= 1) {
				writeBitCyfral(CYFRAL & bitmask, Tj1, Tj0);					//sending start nibble
			}
			for (i = 0; i < 4; i++) {
				for (bitmask = 128; bitmask; bitmask >>= 1) {				//reading nibble from MSB
					writeBitCyfral(buf[i] & bitmask, Tj1, Tj0);
				}
			}
		}	break;
	}
	case METAKOM: {
		pMode(pin_comparator, OUTPUT);										//sending synchronise bit log 0
		(buf[3] & 1) ? delayUs(Tj1) : delayUs(Tj0);
		for (byte retry = 0, bitmask, i; retry < emulRetry; retry++) {
			pMode(pin_comparator, OUTPUT);
			delayUs(T.t1);													//sending synchronise bit log 0
			for (bitmask = 0b100; bitmask; bitmask >>= 1) {
				writeBitMetakom(METAKOM & bitmask, Tj1, Tj0);				//sending start nibble
			}
			for (i = 0; i < 4; i++) {
				for (bitmask = 128; bitmask; bitmask >>= 1) {
					writeBitMetakom(buf[i] & bitmask, Tj1, Tj0);
				}
			}
		}
		pMode(pin_comparator, INPUT);
	}
	}
	EMULATE_HIGH_OFF(pin_data);
	clearVars();
}

inline void Keysniffer::writeBitCyfral(bool bit, const byte& Tj1, const byte& Tj0) {
	pMode(pin_comparator, OUTPUT);	// Start high current consumption	
	if (bit) {
		delayUs(T.ti1);
		pMode(pin_comparator, INPUT);
		delayUs(Tj1);
	}
	else {
		delayUs(T.ti0);
		pMode(pin_comparator, INPUT);  // End high current consumption
		delayUs(Tj0);
	}
}

inline void Keysniffer::writeBitMetakom(bool bit, const byte& Tj1, const byte& Tj0) {
	pMode(pin_comparator, INPUT);		// End high current consumption
	if (bit) {
		delayUs(T.ti1);
		pMode(pin_comparator, OUTPUT);
		delayUs(Tj1);
	}
	else {
		delayUs(T.ti0);
		pMode(pin_comparator, OUTPUT);  // Start high current consumption
		delayUs(Tj0);
	}
}

inline void Keysniffer::clearVars() {
	memset(&T, 0, sizeof(T));
}
