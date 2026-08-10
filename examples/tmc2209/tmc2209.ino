/**
 * @file      tmc2209.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2026  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2026-08-10
 * @note      The T-Motor JST 6-pin V+ GND must be powered; otherwise, communication will fail.
 */

 #include <Arduino.h>
#include <TMCStepper.h>

#define R_SENSE 0.11f

#define EN_PIN 2
#define DIR_PIN 18
#define STEP_PIN 23

#define CLK_PIN 19
#define SPREAD_PIN 4

#define SW_RX 26
#define SW_TX 27

#define SERIAL_BAUD 115200
#define TMC2209_VERSION 0x21
#define DRIVER_COUNT 4

HardwareSerial SerialDriver(1);
TMC2209Stepper drivers[DRIVER_COUNT] = {
    TMC2209Stepper(&SerialDriver, R_SENSE, 0b00),
    TMC2209Stepper(&SerialDriver, R_SENSE, 0b01),
    TMC2209Stepper(&SerialDriver, R_SENSE, 0b10),
    TMC2209Stepper(&SerialDriver, R_SENSE, 0b11),
};

static TMC2209Stepper *activeDriver = nullptr;
static uint8_t activeAddress = 0;

static uint32_t testCount = 0;
static uint32_t passCount = 0;
static uint32_t failCount = 0;

static void printHex32(const char *label, uint32_t value)
{
    Serial.print(label);
    Serial.print("0x");
    Serial.println(value, HEX);
}

static void printAddress(uint8_t address)
{
    Serial.print("0b");
    Serial.print((address >> 1) & 0x01);
    Serial.print(address & 0x01);
}

static bool checkCommunication(TMC2209Stepper &driver, uint8_t address, bool verbose)
{
    const uint32_t ioin = driver.IOIN();
    const uint8_t version = driver.version();
    const uint8_t ifcntBefore = driver.IFCNT();

    driver.toff(0);
    driver.pdn_disable(true);
    driver.mstep_reg_select(true);
    driver.microsteps(16);

    const uint8_t ifcntAfter = driver.IFCNT();
    const bool versionOk = version == TMC2209_VERSION;
    const bool ifcntChanged = ifcntAfter != ifcntBefore;
    const bool passed = versionOk && ifcntChanged;

    if (!verbose) {
        return passed;
    }

    testCount++;
    if (passed) {
        passCount++;
    } else {
        failCount++;
    }

    Serial.println();
    Serial.print("TMC2209 UART test #");
    Serial.println(testCount);
    Serial.print("Address: ");
    printAddress(address);
    Serial.println();
    printHex32("IOIN: ", ioin);

    Serial.print("VERSION: 0x");
    Serial.print(version, HEX);
    Serial.print(versionOk ? " OK" : " ERROR");
    Serial.println(" (expected 0x21)");

    Serial.print("IFCNT: ");
    Serial.print(ifcntBefore);
    Serial.print(" -> ");
    Serial.print(ifcntAfter);
    Serial.println(ifcntChanged ? " OK" : " ERROR");

    Serial.print("Pins ENN/MS1/MS2/PDN_UART/STEP/SPREAD/DIR: ");
    Serial.print(driver.enn());
    Serial.print('/');
    Serial.print(driver.ms1());
    Serial.print('/');
    Serial.print(driver.ms2());
    Serial.print('/');
    Serial.print(driver.pdn_uart());
    Serial.print('/');
    Serial.print(driver.step());
    Serial.print('/');
    Serial.print(driver.spread_en());
    Serial.print('/');
    Serial.println(driver.dir());

    Serial.print("Result: ");
    Serial.println(passed ? "PASS" : "FAIL");
    Serial.print("Summary pass/fail: ");
    Serial.print(passCount);
    Serial.print('/');
    Serial.println(failCount);

    if (!passed) {
        Serial.println("Check UART wiring, RX/TX pins, driver address MS1/MS2, VM/VIO power, and common GND.");
    }

    return passed;
}

static bool scanDriverAddress()
{
    Serial.println();
    Serial.println("Scanning TMC2209 UART address...");
    Serial.println("Datasheet: address bit0=MS1_AD0, bit1=MS2_AD1. MS1=HIGH and MS2=HIGH means address 0b11.");
    Serial.print("UART_RX idle level before scan: ");
    Serial.println(digitalRead(SW_RX) ? "HIGH" : "LOW");

    activeDriver = nullptr;

    for (uint8_t address = 0; address < DRIVER_COUNT; address++) {
        TMC2209Stepper &driver = drivers[address];
        driver.begin();
        delay(20);

        const uint32_t ioin = driver.IOIN();
        const uint8_t version = driver.version();
        const uint8_t ifcntBefore = driver.IFCNT();
        driver.pdn_disable(true);
        const uint8_t ifcntAfter = driver.IFCNT();
        const bool found = (version == TMC2209_VERSION) && (ifcntAfter != ifcntBefore);

        Serial.print("Address ");
        printAddress(address);
        Serial.print(": IOIN=0x");
        Serial.print(ioin, HEX);
        Serial.print(" VERSION=0x");
        Serial.print(version, HEX);
        Serial.print(" IFCNT=");
        Serial.print(ifcntBefore);
        Serial.print("->");
        Serial.print(ifcntAfter);
        Serial.println(found ? " FOUND" : "");

        if (found && activeDriver == nullptr) {
            activeDriver = &driver;
            activeAddress = address;
        }
    }

    if (activeDriver == nullptr) {
        Serial.println("No TMC2209 response found on addresses 0b00..0b11.");
        Serial.println("Scan stopped. Reset the board to scan again.");
        Serial.println("Check PDN_UART idle level, UART_RX/UART_TX routing, VM/VIO power, and common GND.");
        Serial.println("The ST 6 pin V+ GND must be connected to a 9~12V voltage for the TMC2209 to work.");
        return false;
    }

    Serial.print("Active address: ");
    printAddress(activeAddress);
    Serial.println();
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println();
    Serial.println("TMC2209 UART communication test");
    Serial.println("USB Serial: 115200");
    Serial.println("TMC UART: Serial1 RX=26 TX=27");

    pinMode(CLK_PIN, OUTPUT);
    pinMode(SPREAD_PIN, OUTPUT);
    digitalWrite(CLK_PIN, LOW);
    digitalWrite(SPREAD_PIN, HIGH);

    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(SW_RX, INPUT_PULLUP);
    digitalWrite(EN_PIN, LOW);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    SerialDriver.begin(SERIAL_BAUD, SERIAL_8N1, SW_RX, SW_TX);
    delay(100);

    scanDriverAddress();
}

void loop()
{
    if (activeDriver == nullptr) {
        delay(1000);
        return;
    }

    const bool passed = checkCommunication(*activeDriver, activeAddress, true);
    delay(1000);
}
