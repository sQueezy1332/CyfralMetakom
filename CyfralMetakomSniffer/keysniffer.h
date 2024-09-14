#pragma once
#pragma message "Изи кудря"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif
#include <Arduino.h>
#include <GyverIO.h>
#define uS						micros()
#define mS						millis()
#define delayUs(x)				delayMicroseconds(x)
#define delayMs(x)				delay(x)
#define pInit(pin, val)			gio::init(pin, val)
#define pMode(pin, val)			gio::mode(pin, val)
#define dWrite(pin, val)		gio::write(pin, val)
#define dRead(pin)				gio::read(pin)
#define EMULATE_HIGH_ON(pin)	pMode(pin, OUTPUT)
#define EMULATE_HIGH_OFF(pin)	pMode(pin, INPUT)
#if defined(__AVR__)
	#define VOLTAGE_MEASURING
#define wdr					asm("wdr")
#define COMP				(ACSR & bit(ACO))
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
#elif defined(ESP32)	//???
#define	COMP				1
#define wdr					asm("nop")
#define cyclestoIteraion 60
#else 
#error "Unsupported Architecture"
#endif
#define WAIT_RETRY_COUNT	(microsecondsToClockCycles(20) / cyclestoIteraion)
typedef uint8_t byte;
typedef uint16_t word;
typedef uint32_t dword;

extern volatile byte flagInterrupt;
#ifdef VOLTAGE_MEASURING
extern bool flagAdcFirstConv;
extern word avgVoltage;
#endif
extern const byte pin_comparator, pin_data;
//template <byte PIN_DATA, byte PIN_COMP> 
class Keysniffer
{
public:
	Keysniffer();
	bool KeyDetection(byte (&buf)[8]);
	bool Metakom(byte(&buf)[8]);
	bool Cyfral(byte(&buf)[8]);
	void Emulate(const byte buf[], byte keyType, byte emulRetry = 10);
#ifdef VOLTAGE_MEASURING
	word Kalman();
#endif
	typedef enum : uint8_t {
		NO_ERROR = 0,
		CYFRAL = 0b0001,
		METAKOM  = 0b010,
		ERROR_DUTY_LOW_CYFRAL, 
		ERROR_DUTY_HIGH_CYFRAL,
		ERROR_PERIOD_CYFRAL,
		ERROR_NIBBLE_CYFRAL,
		ERROR_DUTY_HIGH_METAKOM,
		ERROR_DUTY_LOW_METAKOM,
		ERROR_PERIOD_METAKOM,
		ERROR_VERY_LONG_LOW,
		ERROR_SYNC_BIT,
		ERROR_START_DUTY_HIGH,
		ERROR_NOT_RECOGNIZED,
	} err_t;
	err_t error;
protected:
	inline bool recvBitCyfral();
	inline bool recvBitMetakom();
	inline void writeBitCyfral(bool bit, const byte& Tj1, const byte& Tj0);
	inline void writeBitMetakom(bool bit, const byte& Tj1, const byte& Tj0);
	bool comparator();
	inline void clearVars();
	struct {
		word t1 = 0;		// Average full period log 1
		word t0 = 0;		// Average full period log 0
		word ti1 = 0;		// Interval of first period - dutyLow (Cyfral) or dutyHigh (Metakom) for logical 1
		word ti0 = 0;		// Interval of first period - dutyLow (Cyfral) or dutyHigh (Metakom) for logical 0
	} T;
	byte period = 0;	// Full period
	byte dutyLow = 0;	// Interval of high current consumption -  log 0
	byte dutyHigh = 0;	// Interval of low current consumption - log 1
};
