#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
	extern unsigned long micros(void);
	//extern void delayMicroseconds(double);
#ifdef __cplusplus
	}
#endif
#define PERIOD_MEASURE
#define uS					micros()
//#define delayUs(x)			delayMicroseconds(x)
#define _WEAK __attribute__((weak))
#if not defined unlikely
#define likely(x)    __builtin_expect(!!(x), 1)
#define unlikely(x)  __builtin_expect(!!(x), 0)
#endif // !unlikely
typedef unsigned char byte;

extern void delayUs(size_t);
extern void emul_low_level();
extern void emul_high_level();
extern bool comparator();

class Keysniffer
{
public:
	Keysniffer() { /*init_io();*/ };
	byte KeyDetection(byte (&buf)[8]);
	byte Metakom(byte buf[8]);
	byte Cyfral(byte buf[8]);
	void Emulate(const byte buf[], byte keyType, byte emulRetry = 10);
	typedef enum : byte {
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
	void writeBitCyfral(size_t Ti, size_t Tj);
	void writeBitMetakom(size_t Ti, size_t Tj);
	size_t period = 0;	// Full period
	byte dutySecond = 0;
	byte dutyFirst = 0;
};
