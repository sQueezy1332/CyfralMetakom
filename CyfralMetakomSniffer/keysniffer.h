#pragma once
#pragma message "Изи кудря"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x, ...) Serial.print(x, ##__VA_ARGS__)
#define DEBUGLN(x, ...) Serial.println(x, ##__VA_ARGS__)
#else
#define DEBUG(x, ...)
#define DEBUGLN(x, ...)
#endif
#include <Arduino.h>
#define uS						micros()
#define mS						millis()
#define delayUs(x)				delayMicroseconds(x)
#define delayMs(x)				delay(x)
#define _WEAK __attribute__((weak))
#if not defined unlikely
#define likely(x)    __builtin_expect(!!(x), 1)
#define unlikely(x)  __builtin_expect(!!(x), 0)
#endif // !unlikely
#define PERIOD_MEASURE

extern void emul_low_level();
extern void emul_high_level();
extern bool comparator();

class Keysniffer
{
public:
	Keysniffer() { /*init_io();*/ };
	byte KeyDetection(byte (&buf)[SIZE]);
	byte Metakom(byte buf[SIZE]);
	byte Cyfral(byte buf[SIZE]);
	void Emulate(const byte buf[], byte keyType, byte emulRetry = 10);
	typedef enum : uint8_t {
		NO_ERROR = 0,
		CYFRAL = 0b0001,
		METAKOM  = 0b010,
		ERROR_SYNC_BIT,
		ERROR_VERY_LONG_LOW,
		ERROR_START_NIBBLE,
		ERROR_DUTY_LOW_CYFRAL, 
		ERROR_DUTY_HIGH_CYFRAL,
		ERROR_PERIOD_CYFRAL,
		ERROR_NIBBLE_CYFRAL,
		ERROR_DUTY_HIGH_METAKOM,
		ERROR_DUTY_LOW_METAKOM,
		ERROR_PERIOD_METAKOM,
		ERROR_PARITY_METAKOM,
		ERROR_START_DUTY_HIGH,
		ERROR_NOT_RECOGNIZED,
	} err_t;
protected:
	byte recvBitMetakom();
	byte recvBitCyfral();
	void writeBitCyfral(byte Ti, byte Tj);
	void writeBitMetakom(byte Ti, byte Tj);
	size_t period = 0;	// Full period
	byte dutySecond = 0;
	byte dutyFirst = 0;
};
