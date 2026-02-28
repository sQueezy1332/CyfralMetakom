#pragma message "Изи кудря"
//#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
//#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include "keysniffer.h"

byte Keysniffer::KeyDetection(byte(&buf)[8]) {
	if (comparator()) { return -1; } //wait until high signal is start
	auto timer = uS;
	while (!comparator()) {			//Try read METAKOM synchronise bit log 0
		if (uS - timer > 450) {		//50 - 230 datasheet ~450 for last bit + synchro
			timer = uS;
			while (!comparator()) {
				if (uS - timer > 200000) {
					return ERROR_VERY_LONG_LOW;
				}
			}
			return ERROR_SYNC_BIT;
		}
	}
	size_t startPeriod = uS - timer + dutySecond;
	if (recvBitMetakom() == 1) return ERROR_START_NIBBLE; //0b1xx == error
	byte startNibble = 0;
	for (byte bitmask = 0b10; bitmask; bitmask >>= 1) {
		if (byte ret = recvBitMetakom()) {
			if (unlikely(ret > 1)) return ret;
			startNibble |= bitmask;
		}
	}
	if (startNibble > METAKOM) return ERROR_START_NIBBLE;
	if (startNibble == METAKOM && startPeriod + 10 >= period) {
		return (Metakom(buf));
	}//last bit could be volatile if Cyfral //b000 or b001
	timer = uS;
	while (comparator()) {				//wait 1 duty cycle for last half bit start nibble (Cyfral?) 
		if (uS - timer > 200) {
			return ERROR_START_DUTY_HIGH;
		}
	}
	if ((uS - timer) > dutySecond || likely(recvBitCyfral() <= 1)) {  //duty low from previos read bit Metakom or b0_0001
		return Cyfral(buf);
	}
	return ERROR_NOT_RECOGNIZED;
}

byte Keysniffer::Metakom(byte buf[]) {
	byte count1 = 0, count0 = 0, i, result, bitmask;
#ifdef PERIOD_MEASURE
	size_t T1 = 0;		// Average full period log 1
	size_t T0 = 0;		// Average full period log 0
	size_t Ti1 = 0;		// Interval of first period for 1
	size_t Ti0 = 0;		// Interval of first period for 0
#endif // PERIOD_MEASURE
	for (i = 0; i < 4; i++) {
		for (result = 0, bitmask = 128; bitmask; bitmask >>= 1) {
			if (byte ret = recvBitMetakom()) {
				if (unlikely(ret > 1)) {
					if ((bitmask == 1) && (i == 3)) {
						if (dutyFirst < (period >> 1)) goto last_bit_zero;
					}
					else return ret;
				}
				result |= bitmask;
				T1 += period;
				Ti1 += dutyFirst;
				count1++;
			}
			else {
last_bit_zero:
				T0 += period;
				Ti0 += dutyFirst;
				count0++;
			}
		}
		if (count1 & 1) {
			return ERROR_PARITY_METAKOM;
		}
		buf[i] = result;
	}
	if (count1) {
		buf[4] = T1 / count1;		// division  for Period 1
		buf[6] = Ti1 / count1;
	} else {
		buf[4] = 0;
		buf[6] = 0;
	}
	if (count0) {				//checking for divider not be zero
		buf[5] = T0 / count0;	// division  for Period 0
		buf[7] = Ti0 / count0;
	}
	else {
		buf[5] = 0;
		buf[7] = 0;
	}
	return NO_ERROR;
}

byte Keysniffer::recvBitMetakom() {
	auto timer = uS;
	while (comparator()) {
		if (uS - timer > 200) { return ERROR_DUTY_HIGH_METAKOM; }
	}
	{ auto t = uS;
	dutyFirst = t - timer;
	timer = t; }
	while (!comparator()) {
		if (uS - timer > 160) {
			dutySecond = 160;		//may be synchronise bit
			return ERROR_DUTY_LOW_METAKOM;
		}
	}
	dutySecond = uS - timer;
	size_t _period = dutySecond + dutyFirst;
	if (_period < 50) { return ERROR_PERIOD_METAKOM; }
	period = _period;
	return dutyFirst > dutySecond;
}

byte Keysniffer::recvBitCyfral() {
	auto timer = uS;
	while (!comparator()) {
		if (uS - timer > 200) {
			return ERROR_DUTY_LOW_CYFRAL;
		}
	}
	{ auto t = uS;
	dutyFirst = t - timer;
	timer = t; }
	while (comparator()) {
		if (uS - timer > 200) {
			dutySecond = 200;
			return ERROR_DUTY_HIGH_CYFRAL;
		}
	}
	dutySecond = uS - timer;
	size_t _period = dutySecond + dutyFirst;
	if(_period < 50) {
		return ERROR_PERIOD_CYFRAL;
	}
	period = _period;
	return dutySecond > dutyFirst;
}

byte Keysniffer::Cyfral(byte buf[]) {
	again:
#ifdef PERIOD_MEASURE
	size_t T1 = 0;		// Average full period log 1
	size_t T0 = 0;		// Average full period log 0
	size_t Ti1 = 0;		// Interval of first period - (Cyfral) for 1
	size_t Ti0 = 0;		// Interval of first period - (Cyfral) for 0
#endif // PERIOD_MEASURE
	for (byte i = 0, nibble = 0, bitmask; i < 4; ++i) {
		for (bitmask = 0b1000; bitmask; bitmask >>= 1) {
			if (byte ret = recvBitCyfral()) {
				if (unlikely(ret > 1)) return ret;
				nibble |= bitmask;
				T1 += period;
				Ti1 += dutyFirst;
			}
			else {
				T0 += period;
				Ti0 += dutyFirst;
			}
		} //Serial.println(nibble, HEX);
		switch (nibble & 0x0F) {
		case 0x7: case 0xB: case 0xD:case 0xE:
			if (nibble & 0xF0) {
				buf[i] = nibble;  //writing nibble from MSB
				nibble = 0;
				continue;
			}
			nibble <<= 4;
			continue;
		case 0x1:
			goto again;
		default:
			return ERROR_NIBBLE_CYFRAL;
		}
	} //fast division by 24 = (65536 / (200 * 32) - 1) * 24 == 221.76; x = (2 ^ 8) / 24 + 1 = 11,66...
	buf[4] = (T1 * 11) >> 8;
	buf[6] = (Ti1 * 11) >> 8;
	buf[5] = T0 >> 3;			// division by 8
	buf[7] = Ti0 >> 3;
	return NO_ERROR;
}

void Keysniffer::Emulate(const byte buf[], byte keyType, byte emulRetry) {
#ifdef PERIOD_MEASURE
	size_t T1 = 0;		// Average full period log 1
	size_t T0 = 0;		// Average full period log 0
	size_t Ti1 = 0;		// Interval of first period
	size_t Ti0 = 0;		// Interval of first period
#endif // PERIOD_MEASURE
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
	//EMULATE_HIGH_ON(pin_data);
	switch (keyType) {
	case CYFRAL: {
		for (byte retry = 0, bitmask, i; retry < emulRetry; retry++) {
			for (bitmask = 0b1000; bitmask; bitmask >>= 1) {
				if (CYFRAL & bitmask) { //sending start nibble
					writeBitCyfral(Ti1, Tj1);
				}
				else {
					writeBitCyfral(Ti0, Tj0);
				}
			}
			for (i = 0; i < 4; i++) {
				for (bitmask = 128; bitmask; bitmask >>= 1) {				//sending nibble from MSB
					if (buf[i] & bitmask) {
						writeBitMetakom(Ti1, Tj1);
					}
					else {
						writeBitMetakom(Ti0, Tj0);
					}
				}
			}
		}	break;
	}
	case METAKOM: {
		emul_low_level();									//sending synchronise bit log 0
		(buf[3] & 1) ? delayUs(Tj1) : delayUs(Tj0);
		for (byte retry = 0, bitmask, i; retry < emulRetry; retry++) {
			emul_low_level();
			delayUs(T1);													//sending synchronise bit log 0
			for (bitmask = 0b100; bitmask; bitmask >>= 1) {
				if (METAKOM & bitmask) { //sending start nibble
					writeBitMetakom(Ti1, Tj1);
				}
				else {
					writeBitMetakom(Ti0, Tj0);
				}				
			}
			for (i = 0; i < 4; i++) {
				for (bitmask = 128; bitmask; bitmask >>= 1) {
					if (buf[i] & bitmask) {
						writeBitMetakom(Ti1, Tj1);
					}
					else {
						writeBitMetakom(Ti0, Tj0);
					}
					
				}
			}
		}
		emul_high_level();
	}
	}
	//EMULATE_HIGH_OFF(pin_data);
}

void Keysniffer::writeBitCyfral(size_t Ti, size_t Tj) {
	emul_low_level();	// Start high current consumption	
	delayUs(Ti);
	emul_high_level();		// End high current consumption
	delayUs(Tj);
}

void Keysniffer::writeBitMetakom(size_t Ti, size_t Tj) {
	emul_high_level();		// End high current consumption
	delayUs(Ti);
	emul_low_level();		// Start high current consumption
	delayUs(Tj);
}

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
