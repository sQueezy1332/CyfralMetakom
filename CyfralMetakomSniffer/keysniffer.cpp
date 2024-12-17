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

bool Keysniffer::KeyDetection(byte(&buf)[SIZE]) {
	register byte startNibble, bitmask;
	word startPeriod;
	dword tempVoltage;decltype(uS) timer;
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
		clearVars();
		if (comparator()) { continue; } //wait until high signal is start
		timer = uS;		
		wdr;
		while (!comparator()) {			//Try read METAKOM synchronise bit log 0
			if (uS - timer > 450) {		//50 - 230 datasheet ~450 for last bit + synchro
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
		startPeriod = uS - timer + dutySecond;
		for (bitmask = 0b100, startNibble = 0; bitmask; bitmask >>= 1) {
			if (recvBitMetakom())
				startNibble |= bitmask;
			if (startNibble > METAKOM || error)
				goto Start;
		}
		if (startNibble == METAKOM && startPeriod + 5 >= period) {
			if (Metakom(buf)) return METAKOM;
			DEBUG(error);
			continue;
		}//last bit could be volatile if Cyfral //b000 or b001
		timer = uS;
		while (comparator()) {				//wait 1 duty cycle for last half bit start nibble (Cyfral?) 
			if (uS - timer > 200) {
				//error = ERROR_START_DUTY_HIGH; DEBUG(error);
				goto Start;
			}
		}
		if ((uS - timer) > dutySecond || !recvBitMetakom(false)) {  //duty low from previos read bit Metakom or b0_0001
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

bool Keysniffer::Metakom(byte(&buf)[SIZE]) {
	register byte count1 = 0, count0 = 0, i, BYTE, bitmask;
	for (i = 0; i < 4; i++) {
		for (BYTE = 0, bitmask = 128; bitmask; bitmask >>= 1) {
			if (recvBitMetakom(true)) {
last_bit:
				BYTE |= bitmask;
				T1 += period;
				Ti1 += dutyFirst;
				count1++;
			}
			else {
				if (error) {
					if ((bitmask == 1) && (i == 3)) {
						if (dutyFirst > (period >> 1)) goto last_bit;
					}
					else return false;
				}
				T0 += period;
				Ti0 += dutyFirst;
				count0++;
			}
		}
		if (count1 & 1) {
			error = ERROR_PARITY_METAKOM;
			return false;
		}
		buf[i] = BYTE;
	}
	buf[4] = T1 / count1;		// division  for Period 1
	buf[6] = Ti1 / count1;
	if (count0) {				//checking for divider not be zero
		buf[5] = T0 / count0;	// division  for Period 0
		buf[7] = Ti0 / count0;
	} else {
		buf[5] = 0;
		buf[7] = 0;
	}
	return true;
}

bool Keysniffer::recvBitMetakom(const bool state) {
	auto timer = uS;
	decltype(timer) t;
	while (comparator() == state) {
		if (uS - timer > 200) {
			error = ERROR_DUTY_HIGH_METAKOM;
			return false;
		}
	}
	t = uS;
	dutyFirst = t - timer;
	timer = t;
	while (comparator() == !state) {
		if (uS - timer > 160) {
			dutySecond = 160;		//may be synchronise bit
			error = ERROR_DUTY_LOW_METAKOM;
			return false;		
		}
	}
	dutySecond = uS - timer;
	if ((period = dutySecond + dutyFirst) < 50) {
		error = ERROR_PERIOD_METAKOM;
		return false;
	}
	return (dutyFirst > dutySecond);
}

bool Keysniffer::Cyfral(byte (&buf)[SIZE]) {
again:
	for(byte i = 0, nibble = 0, bitmask; i < 4;++i) {
		for (bitmask = 0b1000; bitmask; bitmask >>= 1) {
			if (!recvBitMetakom(false)) {
				nibble |= bitmask;
				T1 += period;
				Ti1 += dutyFirst;
				if (error) return false;
			}
			else {
				T0 += period;
				Ti0 += dutyFirst;
			}
		} //Serial.println(nibble, HEX);
		switch (nibble & 0xF) {
		case 0x7: case 0xB: case 0xD:case 0xE:
			if (nibble & 0xF0) {
				buf[i] = nibble;  //writing nibble from MSB
				break;
			}
			nibble <<= 4;
			continue;
		case 0x1:
			goto again;
		default:
			error = ERROR_NIBBLE_CYFRAL;
			return false;
		}
	} //fast division by 24 = (65536 / (200 * 32) - 1) * 24 == 221.76; x = (2 ^ 8) / 24 + 1 = 11,66...
	buf[4] = (T1 * 11) >> 8;
	buf[6] = (Ti1 * 11) >> 8;
	buf[5] = T0 >> 3;			// division by 8
	buf[7] = Ti0 >> 3;
	return true;
}

bool Keysniffer::comparator() {
	static byte prev_state = COMP;
	byte state = COMP;
	if (state != prev_state) {
		auto time = micros();
		while (COMP == state) {
			if (micros() - time > DELAY_COMP) {
				prev_state = state;
				return state;
			}
		}
	}
	return prev_state;
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
		T1 = buf[4];
		T0 = buf[5];
		Ti1 = buf[6];
		Ti0 = buf[7];
	}
	else {
		if (keyType == CYFRAL) { T1 = T0 = 180; Ti1 = 60; Ti0 = 120; }
		else if (keyType == METAKOM) { T1 = T0 = 180; Ti1 = 120; Ti0 = 60; }
		else return;
	}
	const byte Tj1 = T1 - Ti1;
	const byte Tj0 = T0 - Ti0;
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
			delayUs(T1);													//sending synchronise bit log 0
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

void Keysniffer::writeBitCyfral(bool bit, const byte& Tj1, const byte& Tj0) {
	pMode(pin_comparator, OUTPUT);		// Start high current consumption	
	delayUs(bit ? Ti1 : Ti0);
	pMode(pin_comparator, INPUT);		// End high current consumption
	delayUs(bit ? Tj1 : Tj0);
}

void Keysniffer::writeBitMetakom(bool bit, const byte& Tj1, const byte& Tj0) {
	pMode(pin_comparator, INPUT);		// End high current consumption
	delayUs(bit ? Ti1 : Ti0);
	pMode(pin_comparator, OUTPUT);		// Start high current consumption
	delayUs(bit ? Tj1 : Tj0);
}
/*
bool Keysniffer::recvBitCyfral() {
	auto timer = uS;
	decltype(timer) t;
	while (!comparator()) {
		if (uS - timer > 160) {
			error = ERROR_DUTY_LOW_CYFRAL;
			return false;
		}
	}
	dutyFirst = t - timer;
	timer = t;
	while (comparator()) {
		if (uS - timer > 200) {
			error = ERROR_DUTY_HIGH_CYFRAL;
			return false;
		}
	}
	dutyFirst = uS - timer;
	if ((period = uS - timer) < 50) {
		error = ERROR_PERIOD_CYFRAL;
		return false;
	}
	return (dutyFirst > dutySecond);
}*/
/*
bool Keysniffer::comparator() {
	static byte iterator = WAIT_RETRY_COUNT;
	byte old_iterator = 0;
	for (;;) {
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
		register char temp = (iterator - (iterator - old_iterator) * 2) - 1;
		if (temp < 1) { iterator = 0;  return false; }
		else iterator = temp;

	}
}*/
