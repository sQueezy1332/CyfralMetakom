#pragma once
//#pragma GCC diagnostic ignored "-Wconversion-null"
//#include <Arduino.h>
#include <Softwareserial.h>
typedef uint8_t byte;
//#include <string.h> 
//#include <stdlib.h>

#define keylen 8 
#define RESP_BUFLEN 64
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

#define BAUDRATE 9600 //not recommend to use more 19200 if open drain uart is enabled

SoftwareSerial //<1, 0> 
sim(RX, TX);

__attribute__((weak))  extern volatile byte flagInterrupt;
__attribute__((weak))  extern byte flagSmsNotsended;
char respBuf[RESP_BUFLEN] {};
void initSIM();
void resetSIM();

bool waitResponse(size_t timeout = 5000) {
	uint32_t timer = millis();
	do {
		if (sim.available()) {
			byte amount = sim.readBytesUntil('\n', respBuf, RESP_BUFLEN);
			respBuf[amount] = '\0';
			DEBUG(respBuf);
			return true;
		}
	} while (millis() - timer < timeout);
	DEBUG(F("\tTimeout..."));
	return false;
}


bool sendAT(const char* cmd, const char* response, size_t timeout = 3000) { //retry 0 dont waiting
	sim.println(cmd);
	if (!waitResponse(timeout)) return false;
	return !strstr(respBuf, response);
}

bool sendAT(const char* cmd, char response = OK, size_t timeout = 3000) {
	char resp[] { response, '\0' };
	return sendAT(cmd, resp, timeout);
}

bool SendSms(const byte pBase[][keylen], const char* number, word* vArray, byte keysAmount, byte volKeysAmount) {
	if (!sendAT("AT+CCALR?", CONNECT)) {
		resetSIM();
		delay(5000);
		if (!sendAT("AT+CCALR?", CONNECT)) return false;
	}
	sendAT("AT+CSQ", ':');
	char rssi[12];
	strlcpy(rssi, respBuf, sizeof(rssi));
	sim.print("AT+CMGS=\""); sim.print(number);
	if (!sendAT("\"", '>')) {
		resetSIM();
		return false;
	}
	for (byte i = 0, j = 0; i < keysAmount; i++) {
		while (j < 4) { sim.print(pBase[i][j++], HEX); sim.print(' '); }
		sim.print('|');
		while (j < 7) { sim.print(pBase[i][j++], DEC); sim.print(' '); }
		sim.println(pBase[i][j], DEC); j = 0;
	}
	if (volKeysAmount && vArray != nullptr) {
		sim.print("\nvolKeysAmount = "); sim.println(volKeysAmount);
		for (byte i = 0; i < volKeysAmount; i++) sim.println(vArray[i]);
	}
	if (flagSmsNotsended > 1) { sim.print("flagSmsNotsended = ");  sim.println(flagSmsNotsended, HEX); }
	if (flagInterrupt) { sim.print("flagInterrupt = ");  sim.println(flagInterrupt); }
	sim.print(rssi);
	if (sendAT(SUB, "CMGS")) return true;
	else if (sendAT(ESC, '>')) { 
		if (!waitResponse() || !strchr(respBuf, ERROR)) 
			resetSIM(); 
	}
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
	sim.begin(BAUDRATE);
	sim.setTimeout(100);
	delay(5000);
	if (!sendAT("AT")) sendAT(ESC);
	/*echo off, text mode off, error numeric, AON on, sms text mode, DTMF on */
	sendAT("ATE0V0+CMEE=1;+CLIP=1;+DDET=1,250;+CMGF=1;&W"); //ATE0V1
}
void resetSIM() {
	pinMode(PIN_SIM_RST, OUTPUT);
	delay(100);
	pinMode(PIN_SIM_RST, INPUT);
}

