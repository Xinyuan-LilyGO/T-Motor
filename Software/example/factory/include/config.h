#ifndef __CONFIG_H_
#define __CONFIG_H_

#define WIFISSID "" //"your ssid";
#define WIFIPASS "" //"your password";

#ifdef WIFISSID
#error Please enter WIFI information. If yes, comment this information
#endif

#define MAX_SPEED (180 * steps_per_mm)
//If the pin connection optocoupler is 1 :OUTPUT
//If the pin connection MOS is 0 :INPUT
#define EN_DIR_STEP_OUTPUT 1

#define R_SENSE 0.11f // Match to your driver                         \
                      // SilentStepStick series use 0.11              \
                      // UltiMachine Einsy and Archim2 boards use 0.2 \
                      // Panucatt BSD2660 uses 0.1                    \
                      // Watterott TMC5160 uses 0.075

#define EN_PIN 2    //enable (CFG6)
#define DIR_PIN 18  //direction
#define STEP_PIN 23 //step

#define CLK_PIN 19
#define SPREAD_PIN 4

#define SW_RX 26 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX 27 // TMC2208/TMC2224 SoftwareSerial transmit pin

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define BTN1 36 //NEXT
#define BTN2 34 //ENTER
#define BTN3 35 //MENU

#define SPI_MT_CS 15 //MT6816
#define SPI_CLK 14
#define SPI_MISO 12
#define SPI_MOSI 13

#define IIC_SCL 21
#define IIC_SDA 22

#define iStep 25
#define iDIR 32
#define iEN 33

#endif