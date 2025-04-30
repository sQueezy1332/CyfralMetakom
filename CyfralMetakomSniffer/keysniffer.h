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
	#define PERIOD_MEASURE
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
#define DELAY_COMP (20)
#define WAIT_RETRY_COUNT	(microsecondsToClockCycles(DELAY_COMP) / cyclestoIteraion)
#define WEAK __attribute__((weak))
#ifdef PERIOD_MEASURE
#define SIZE 8
#else
#define SIZE 4
#endif // PERIOD_MEASURE

typedef uint8_t byte;
typedef uint16_t word;
typedef uint32_t dword;

WEAK extern volatile byte flagInterrupt;
#ifdef VOLTAGE_MEASURING
WEAK extern bool flagAdcFirstConv;
WEAK extern word avgVoltage;
#endif
WEAK extern const byte pin_comparator, pin_data;
//template <byte PIN_DATA, byte PIN_COMP> 
class Keysniffer
{
public:
	Keysniffer();
	bool KeyDetection(byte (&buf)[SIZE]);
	bool Metakom(byte(&buf)[SIZE]);
	bool Cyfral(byte(&buf)[SIZE]);
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
		ERROR_PARITY_METAKOM,
		ERROR_VERY_LONG_LOW,
		ERROR_SYNC_BIT,
		ERROR_START_DUTY_HIGH,
		ERROR_NOT_RECOGNIZED,
	} err_t;
	err_t error;
protected:
	bool recvBitMetakom();
	bool recvBitCyfral();
	void writeBitCyfral(bool bit, const byte& Tj1, const byte& Tj0);
	void writeBitMetakom(bool bit, const byte& Tj1, const byte& Tj0);
	WEAK virtual bool comparator();
	void clearVars() { T0 = T1 = Ti0 = Ti1 = 0; };
#ifdef PERIOD_MEASURE
	word T1 = 0;		// Average full period log 1
	word T0 = 0;		// Average full period log 0
	word Ti1 = 0;		// Interval of first period - dutySecond (Cyfral) or dutyFirst (Metakom) for logical 1
	word Ti0 = 0;		// Interval of first period - dutySecond (Cyfral) or dutyFirst (Metakom) for logical 0
#endif // PERIOD_MEASURE
	byte period = 0;	// Full period
	byte dutySecond = 0;// Interval of high current consumption -  log 0
	byte dutyFirst = 0;	// Interval of low current consumption - log 1
};
