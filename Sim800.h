#pragma once
#pragma GCC diagnostic ignored "-Wconversion-null"
#include <Arduino.h>
#include <Softwareserial.h>
#include <string.h> 
#include <stdlib.h>

//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif
#define keylen 8 
#define buflen 40
#define RX 10 //green TX on sim800 push-pull
#define TX 11  //orange RX on sim800 pull-up
#define PIN_SIM_RST 9
#define OK			'0'		// 0x30
#define CONNECT     '1'		// 0x31
#define RING		'2'		// 0x32
#define NOCARRIER	'3'		// 0x33
#define ERROR		'4'		// 0x34
#define NODIALTONE	'6'		// 0x36
#define BUSY		'7'		// 0x37
#define NOANSWER	'8'		// 0x38
#define PROCEEDING	'9'		// 0x39
#define LF			'\n'	// 0xA
#define CR			'\r'	// 0xD
#define TAB			'\t'	// 9
#define SPACE		'\x20'	// 32
#define SUB			"\x1A"	// 26
#define ESC			"\x1B"	// 27

SoftwareSerial //<1, 0> 
sim (RX, TX);

extern volatile byte flagInterrupt;
extern byte flagSmsNotsended;
char respBuf[buflen] = "";
void initSIM();
void resetSIM();

char* waitResponse(byte timeout_sec = 5) {
	uint32_t timeout = millis() + timeout_sec * 1000ul;
	while (millis() < timeout) {
		if (sim.available()) {
			byte amount = sim.readBytes(respBuf, buflen);
			respBuf[amount] = NULL;
			DEBUG(respBuf);
			return respBuf;
		}
	}
	DEBUG(F("\tTimeout..."));
	return nullptr;
}


bool sendAT(const char* cmd, const char* response, byte retry = 3, byte timeout_sec = 3) { //retry 0 dont do waiting
	do sim.println(cmd);
	while (retry && strstr(waitResponse(timeout_sec), response) == NULL && --retry);
	return retry;
}

bool sendAT(const char* cmd, char response = OK, byte retry = 3, byte timeout_sec = 3) {  
	do sim.println(cmd);
	while (retry && strchr(waitResponse(timeout_sec), response) == NULL && --retry);
	return retry;
}

bool SendSms(const byte pBase[][keylen], const char* send_sms, word* vArray, const byte& keysAmount, const byte& volKeysAmount) {
	if (!sendAT("AT+CCALR?", CONNECT)) {
		resetSIM();
		delay(5000);
		if (!sendAT("AT+CCALR?", CONNECT)) return false;
	}
	char rssi[11] {};
	sendAT("AT+CSQ", ':');
	strlcpy(rssi, respBuf, 11);
	if (!sendAT(send_sms, '>')) { 
		resetSIM();
		return false;
	}
	for (byte i = 0, j = 0; i < keysAmount; i++) {
		while (j < 4) { sim.print(pBase[i][j++], HEX); sim.print(SPACE); }
		sim.print('@');
		while (j < 7) { sim.print(pBase[i][j++], DEC); sim.print(SPACE); }
		sim.println(pBase[i][j], DEC); j = 0;
	}
	if (volKeysAmount && vArray != nullptr) {
		sim.print("\nvolKeysAmount = "); sim.println(volKeysAmount); 
		for (byte i = 0; i < volKeysAmount; i++) sim.println(vArray[i]); 
	}
	if (flagSmsNotsended > 1) { sim.print("flagSmsNotsended = ");  sim.println(flagSmsNotsended, HEX); }
	if (flagInterrupt) { sim.print("flagInterrupt = ");  sim.println(flagInterrupt); }
	sim.print(rssi);
	if (sendAT(SUB, "CMGS", 3, 5)) return true; 
	else if (sendAT(ESC, '>', 1) && !strchr(waitResponse(), ERROR)) resetSIM();
	return false;
}
/*
void available() {
	for (;;) {
		if (sim.available())  Serial.write(sim.read());
		if (Serial.available()) sim.write(Serial.read());
	}
}
*/
void initSIM() {
	sim.begin(9600);
	sim.setTimeout(100);
	delay(5000);
	if (!sendAT("AT")) sendAT(ESC);
	/*echo off, text mode off, error numeric, AON on, sms text mode, DTMF on */
	sendAT("ATE0V0+CMEE=1;+CLIP=1;+DDET=1,250;+CMGF=1;&W"); //ATE0V1
}
void resetSIM() {
	pMode(PIN_SIM_RST, OUTPUT);
	delay(100);
	pMode(PIN_SIM_RST, INPUT);
}

