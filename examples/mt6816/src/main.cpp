#include <Arduino.h>
#include "SPI.h"

//MT6816 pins
#define SPI_MT_CS 15 
#define SPI_CLK 14
#define SPI_MISO 12
#define SPI_MOSI 13

SPIClass MT6816;

int MT6816_read(void)
{
    uint16_t temp[2];
    digitalWrite(SPI_MT_CS, LOW);
    MT6816.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE3));
    temp[0] = MT6816.transfer16(0x8300) & 0xFF;
    MT6816.endTransaction();
    digitalWrite(SPI_MT_CS, HIGH);

    digitalWrite(SPI_MT_CS, LOW);
    MT6816.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE3));
    temp[1] = MT6816.transfer16(0x8400) & 0xFF;
    MT6816.endTransaction();
    digitalWrite(SPI_MT_CS, HIGH);
    return (int)(temp[0] << 6 | temp[1] >> 2);
}

void setup()
{
    Serial.begin(115200);
    pinMode(SPI_MT_CS, OUTPUT);
    MT6816.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_MT_CS);
    MT6816.setClockDivider(SPI_CLOCK_DIV4);
}

void loop()
{
    int val = MT6816_read();
    Serial.println(val); 
    delay(100);
}


