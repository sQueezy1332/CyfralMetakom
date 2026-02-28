#pragma once
#pragma GCC diagnostic ignored "-Woverflow"
#pragma GCC diagnostic ignored "-Wparentheses"
#define DEBUG_ENABLE
#include <Arduino.h>
#include <keysniffer.h>
#include "Sim800.h"

#ifdef __AVR__
#define wdt_off() bitClear(WDTCSR, WDIE)
#define wdt_on() bitSet(WDTCSR, WDIE)
#if defined (__LGT8FX8P__)
#include <WDT.h>
#define wdt_init() wdt_ienable(WTO_32S)
#define PIN_DATA			14
#define PIN_COMP			15
#elif defined __AVR_ATmega328P__
#define wdt_init() WDTCSR |= _BV(WDCE) | _BV(WDE); WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0) // ~8450ms 
#define PIN_DATA			14
#define PIN_COMP			15
#define PIN_REF_PULLUP		3
const byte pin_pullup = PIN_REF_PULLUP;
#endif // __AVR__
#elif defined(ESP32)
#define PIN_DATA			25
#define	PIN_PULLUP			33
#endif

#include <EEPROM.h>
#include <GyverIO.h>
#define kLIMIT 100
#define keylen 8 
#define limitMem kLIMIT	//(E2END/keylen) //127 
#define voltageFirstbyte (limitMem * keylen)
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
#define SEND_SMS_PHONE PHONE_NUMBER_1
#define ConvAdcToVolts(x)	((x) / (adcMaxValue / refVoltage))
#define ConstVOLTAGE        4.757
#define ConstRESISTOR       1454
#define ResWiper            42
#define ResBA               10000
#define CalcResistance(v)	((v) / ((ConstVOLTAGE - (v)) / ConstRESISTOR))
#define CalcDx(r)           int(256 - (float((r) - ResWiper) / ResBA * 256))
#define wdr()				asm("wdr")
#define COMP()				(ACSR & bit(ACO))
#define DELAY_COMP (20)
#define WAIT_RETRY_COUNT	(microsecondsToClockCycles(DELAY_COMP) / cyclestoIteraion)
#define mS					millis()

#define pInit(pin, val)			gio::init(pin, val)
#define pMode(pin, val)			gio::mode(pin, val)
#define dWrite(pin, val)		gio::write(pin, val)
#define dRead(pin)				gio::read(pin)
#define EMULATE_HIGH_ON(pin)	pMode(pin, OUTPUT)
#define EMULATE_HIGH_OFF(pin)	pMode(pin, INPUT)

#define VOLTAGE_MEASURING
#ifdef VOLTAGE_MEASURING
#if defined(__AVR__)
#if defined(__LGT8FX8P__)
#define compRefVoltage		1.61
#define refVoltage			4.075 //2.035
#define adcMaxValue			0xFFF
#define cyclestoIteraion	11ul
#elif defined(__AVR_ATmega328P__)
#define refVoltage			5.004
#define adcMaxValue			0x3FF
#define adcAvgDivider		5
#define cyclestoIteraion	15ul
extern const byte pin_pullup;
#endif
#define adcAvgof			6	//2^x
#define adcTriggerValue		int(3.9 / refVoltage * adcMaxValue) //3919
#elif defined(ESP32)	//TODO
#define	COMP				1
#define wdr()				
#define cyclestoIteraion 60
#else 
#error "Unsupported Architecture"
#endif
static bool voltage_measure();
//bool flagAdcFirstConv = true;
word avgVoltage = 0;
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
uint32_t timestamp, period;
byte flagSmsNotsended = 0, writedKeys = 0, keyReaded = 0;

const byte pin_comparator = PIN_COMP; 
const byte pin_data = PIN_DATA;
void delayUs(size_t us) { delayMicroseconds(us); }
void emul_low_level() { pMode(pin_comparator, OUTPUT); };
void emul_high_level() { pMode(pin_comparator, INPUT); };

static Keysniffer obj;
static byte kArray[kLIMIT][keylen] {};
static void emulateKeys(byte keyNO, byte keyType);
static void sortingArray(byte (&buf)[kLIMIT][keylen] = kArray, byte& count = keyReaded);
static void writeKeys();
static void sendKeys(const byte&);
static void clearArray();
static void clearMemory();
static void printkeys();
static void getfromeeprom();
static void firstStart();
static void HandleCall();
static void disable_wdt() { wdt_off(); }
static void enable_wdt() { wdt_on(); }
static void init_io();

template<void(*first)() = disable_wdt, void(*second)() = enable_wdt>
struct AutoFun {
	AutoFun() { first(); };
	~AutoFun() { second(); };
	AutoFun(const AutoFun&) = delete;
	AutoFun& operator=(const AutoFun&) = delete;
};

