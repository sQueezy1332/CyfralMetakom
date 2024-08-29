#pragma once
#pragma GCC diagnostic ignored "-Woverflow"
#pragma GCC diagnostic ignored "-Wparentheses"
#include <Arduino.h>
#include <keysniffer.h>
#include "Sim800.h"
#if defined (__LGT8FX8P__)
#include <WDT.h>
#define wdt_enable wdt_ienable(WTO_32S)
#define PIN_DATA			14
#define PIN_COMP			15
#elif defined __AVR_ATmega328P__
#define wdt_enable WDTCSR |= _BV(WDCE) | _BV(WDE); WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0) // ~8450ms 
#define PIN_DATA			14
#define PIN_COMP			15
#define PIN_REF_PULLUP		3
const byte pin_pullup = PIN_REF_PULLUP;
#elif defined(ESP32)
#define PIN_DATA			25
#define	PIN_PULLUP			33
#endif
#include <EEPROM.h>
#define LIMIT 100
#define keylen 8 
#define limitKeys 100	//(E2END/keylen) //127 
#define voltageFirstbyte (limitKeys * keylen)
#define writedKeysByte (E2END)
#define smsNotsendedByte  (E2END - 1)
#define voltagekeyWritedbyte  (E2END - 2)
#define readByte(addr) EEPROM.read(addr)
#define writeByte(addr, value) EEPROM.update(addr, value)
#define readKey(addr, ptr) EEPROM.get((addr) * keylen, ptr)
#define writeKey(addr, ptr) EEPROM.put((addr) * keylen, ptr)
#define readBlock(addr, ptr) EEPROM.get((addr), ptr)
#define writeBlock(addr, ptr) EEPROM.put((addr), ptr)
#define PIN_RING 2
#define PHONE_NUMBER_1	"9990000000"
#define PHONE_NUMBER_2	"9990000000"
#define SEND_SMS_PHONE R"(AT+CMGS="+79990000000")"

#define ConvAdcToVolts(x)	((x) / (adcMaxValue / refVoltage))
#define ConstVOLTAGE        4.757
#define ConstRESISTOR       1454
#define ResWiper            42
#define ResBA               10000
#define CalcResistance(v)	((v) / ((ConstVOLTAGE - (v)) / ConstRESISTOR))
#define CalcDx(r)           int(256 - (float((r) - ResWiper) / ResBA * 256))

	#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif
#ifdef VOLTAGE_MEASURING
bool flagAdcFirstConv = true;
word avgVoltage = adcMaxValue;
byte voltagekeyWrited = 0;
word* vArray = nullptr;
#endif

enum flags : uint8_t {
	ZERO,
	RINGCALL,
	KEYREADED,
	SMSNOTSENDED,
	READDISABLE,
};
volatile byte flagInterrupt = ZERO;
uint32_t timestamp;
byte flagSmsNotsended = 0;
byte writedKeys = 0;
byte keyReaded = 0;

const byte pin_comparator = PIN_COMP, pin_data = PIN_DATA;

Keysniffer obj;
byte kArray[LIMIT][keylen] {};
void emulateKeys(byte keyNO, byte keyType);
void sortingArray();
void writeKeys();
void sendKeys(const byte&);
void clearArray();
void clearMemory();
void printkeys();
void getfromeeprom();
void firstStart();
void HandleCall();
