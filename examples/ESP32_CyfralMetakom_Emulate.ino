/*
 Name:		ESP32_CyfralMetakom.ino
 Created:	29-Jan-24 17:25:07
 Author:	sQueezy
*/

#include <keysniffer.h>

#define DEBUG_ENABLE
#define LIMIT 10
#define length 9
#define size_t uint8_t

#define PIN_LED 2
#define PIN_PULLUP 5

const byte pin_comparator = 0xFF, pin_data = 0xFF;

byte pArray[][8]{
 {  0xEE, 0xEE, 0xEE, 0xEE, 180, 180, 60, 120 },
 {  0xD7, 0xE7, 0xBD, 0x7B, 180, 180, 60, 120 },
 {  0x77, 0xBD, 0xEB, 0xED, 180, 180, 60, 120 },
 {  0xAA, 0xE9, 0x47, 0x99, 180, 180, 60, 120 },
 {  0x67, 0x81, 0x00, 0x90, 180, 180, 60, 120 }
};

void setup()
{
    pInit(PIN_LED, OUTPUT);
#ifdef DEBUG_ENABLE
    Serial.begin(115200);
    delay(1);
    DEBUG("Ready");
#endif
    pInit(PIN_PULLUP, OUTPUT); dWrite(PIN_PULLUP, HIGH);
}

void loop()
{
    Keysniffer obj;
    for (;;) {
        while (dRead(27)) {
            dWrite(PIN_LED, HIGH);
            size_t i = 0;
            for (; i < 3; i++) {
                obj.Emulate(pArray[i], Keysniffer::CYFRAL);
            }
            for (; i < 5; i++) {
                obj.Emulate(pArray[i], Keysniffer::METAKOM);
            }
            dWrite(PIN_LED, LOW);
            delay(500);
        }
    }
}


