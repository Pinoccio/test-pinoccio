/*

328p Pins Used:
- RX0/TX0 - testing DUT RX1/TX1
- PD4 - DUT D2
- PD5 - DUT D3
- PD6 - DUT D4
- PD7 - DUT D5
- PB0 - DUT D6
- PB1 - DUT D7
- PB2 - DUT D8

Driver board Pins Used:
- RX0/TX0 - testing USB of DUT
- RX1/TX1 - driving Serial LCD
- MOSI/MISO/SCK - Flashing 256RFR2/16U2
- SS - SS-256RFR2
- D2 - A0 ADC test
- A0 - set AREF to 3.3v or off
- A1-A7 - A1-A7
- D3 - enable power switch (with high-side load switch)
- D4 - enable/disable power through USB port
- D5 - SCL
- D6 - SDA
- D7 - 16U2 Reset
- D8 - 256RFR2 Reset
- BKPK - BKPK
- SDA/SCL - Tied to RGB LED sensor

DUT pins used:
- BKPK - BKPK
- RST - 328p PD3
- SCK/MISO/MOSI/SS - Driver MOSI/MISO/SCK/SS
- RX0/TX0 - Driver RX0/TX0 through USB
- D2-D8 - 328p PD4-PB2
- 3V3 - 328p ADC (PC0)
- VBAT - 328p ADC (PC1)
- RX1/TX1 - 328p RX0/TX0
- SCL/SDA - 328p PC2/PC3
- AREF - Driver D4 (with SPDT switch)
- A0 - Driver D2 (PWM)
- A1-A7 - Driver A1-A7
- BAT+/- - Tied to lipo battery simulator
- Switch pins - Driver D8 (high side load switch)

 ------ SCOUT ------
 + Flash 16U2
   - Read signature
   - Erase chip
   - Write fuses
   - Read fuses
   - Write flash
 
 + Flash 256RFR2
   - Read signature
   - Erase chip
   - Write fuses
   - Read fuses
   - Write EEPROM with incremented ID
   - Read EEPROM
   - Write bootloader flash
   - Write default Scout sketch to flash through USB
 
 + Test reset pin *
   - Assert reset pin
   - Check that new Bitlash header is output
   
 - Test GPIO *
   - Iterate through Bitlash pins as outputs: D2-D8, A0-A7, SCK, MOSI, MISO, SS, RX1/TX1, BKPKBUS
   - Ensure set correctly
 
 - Test RGB LED (using TCS34717FN) TODO
   - Set RGB LED to red, test red is shown
   - Set RGB LED to green, test green is shown
   - Set RGB LED to blue, test blue is shown
 
 + Test power (328p ADC 0-5v)
   - Enable 3V3 pin, test is high
   - Disable 3V3 pin, test is low
   - Enable power through USB port, test VUSB is 5.0v, battery is charging
   - Disable power through USB port, test VUSB is 0.0v, battery is not charging
   - Test battery voltage through fuel gauge
   - Test battery percentage through fuel gauge
 
 - Test mesh
   - Send echo message to driver scout, test message is received and LQI/RSSI are appropriate

*/


#include "programmer.h"
//#include <serialGLCDlib.h>
#include <SPI.h>
#include <Wire.h>
#include <Scout.h>

#define DRIVER_VERSION "1.1"

#define AVR_TESTSUITE_DEBUG
#ifdef AVR_TESTSUITE_DEBUG
#  define TD(x) x
#else
#  define TD(x)
#endif

const int SLAVE_ADDR = 2;

FlashClass DriverFlash(SS, SPI);

//serialGLCD lcd;

#define POWER_SWITCH 3
#define VBAT_SWITCH 4
#define VUSB_SWITCH 5
#define MEGA_256RFR2_RESET 6
#define MEGA_16U2_RESET 7

#define VUSB_ADC "RA0"
#define VBAT_ADC "RA1"
#define VCC_ADC "RA2"

const int startButton = 8;

const float VUSB_MIN = 4.75;
const float VBAT_MIN = 3.7;
const float VCC_MIN = 3.0;
const float OFF_MAX = 1.1;

#define DRIVER_FLASH_CS SS
#define AREF_SWITCH A0
#define JOY_V A5
#define JOY_H A6
#define JOY_SWITCH A7

bool testIsRunning;
bool testFailed;

static NWK_DataReq_t appDataReq;
int pingRSSI;

char wireBuffer[256];
char* cmd = (char*)malloc(32);
int ctr = 0;

int foundSig = -1;

uint8_t eepromVersion = 1;
uint8_t hwVersion = 1;
uint16_t hwFamily = 1000;
const uint32_t HW_SERIAL_ADDR = 0x30000;

const bool RESET_HW_SERIAL = false;
const uint32_t HW_SERIAL_INIT = 0xF4240;    // 1,000,000
//const uint32_t HW_SERIAL_INIT = 0x1E8480; // 2,000,000
//const uint32_t HW_SERIAL_INIT = 0x2DC6C0; // 3,000,000

uint32_t hwSerial;

void setup() {
  uint32_t start = millis();
  
  Wire.begin();
  Serial1.begin(115200);
  addBitlashFunction("i2c.send", (bitlash_function) i2cSend);
  Scout.disableShell();
  Scout.setup();
  
  TD(Serial1.println("- Initialize Test Jig"));
  sendCommand("?", 3);
  readWire();
  if (strncmp((const char*)wireBuffer, DRIVER_VERSION, 3) != 0) {
    TD(Serial1.println("FAIL: Unable to communicate with 328 chip "));
    testFailed = true;
  } else {
    TD(Serial1.println("-- 328 chip ready"));
  }
  
  bool flashFound = false;
  while (millis() - start < 1000) {
    if (DriverFlash.available()) {
      TD(Serial1.println("-- Serial flash chip found"));
      flashFound = true;
      DriverFlash.end();
      break;
    }
  }
  
  if (!flashFound) {
    TD(Serial1.println("FAIL: Serial flash chip not found"));
  }
  
  getSettingsFromFlash();
  
  testJigSetup();
  RgbLed.cyan();
}

void loop() {
  //Scout.loop();
  testJigLoop();
}

void getSettingsFromFlash() {
  TD(Serial1.println("-- Setting up unique ID handler"));
  resetSPIChipSelectPins();
  
  if (RESET_HW_SERIAL == true) {
    TD(Serial1.print("--- Initializing unique ID to: 0x"));
    TD(Serial1.println(HW_SERIAL_INIT, HEX));
    writeHwSerialToFlash(HW_SERIAL_INIT);
  }
  
  TD(Serial1.print("-- Reading HW unique ID from flash: 0x"));
  hwSerial = readHwSerialFromFlash();
  TD(Serial1.println(hwSerial, HEX));
  TD(Serial1.println("--- Done"));
}

void testJigSetup() {
  
  resetHardwarePins();
  
  testIsRunning = false;
  testFailed = false;

  while(Serial.read() != -1);
  
  TD(Serial1.println("--- Scout Test Jig ready to go! ---"));
  //lcd.clearLCD();
  //Serial1.println("Hello");
}

void resetHardwarePins() {
  // disable all switches and chip selects
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  pinMode(POWER_SWITCH, OUTPUT);
  digitalWrite(POWER_SWITCH, LOW);
  pinMode(VBAT_SWITCH, OUTPUT);
  digitalWrite(VBAT_SWITCH, LOW);
  pinMode(VUSB_SWITCH, OUTPUT);
  digitalWrite(VUSB_SWITCH, LOW);
  
  resetSPIChipSelectPins();
  
  pinMode(BACKPACK_BUS, INPUT);
  digitalWrite(BACKPACK_BUS, LOW);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(MOSI, OUTPUT);
  digitalWrite(MOSI, LOW);

  pinMode(startButton, INPUT);
  digitalWrite(startButton, HIGH);
  
  doCommand("i2c.send(\"WD000\")");
  doCommand("i2c.send(\"WD010\")");
  doCommand("i2c.send(\"WD020\")");
  doCommand("i2c.send(\"WD030\")");
  doCommand("i2c.send(\"WD040\")");
  doCommand("i2c.send(\"WD050\")");
  doCommand("i2c.send(\"WD060\")");
  doCommand("i2c.send(\"WD070\")");
  doCommand("i2c.send(\"WD080\")");
  doCommand("i2c.send(\"WD090\")");
  doCommand("i2c.send(\"WD100\")");
  doCommand("i2c.send(\"WD120\")");
  doCommand("i2c.send(\"WD170\")");  
}

void resetSPIChipSelectPins() {
  pinMode(MEGA_256RFR2_RESET, OUTPUT);
  pinMode(MEGA_16U2_RESET, OUTPUT);
  pinMode(DRIVER_FLASH_CS, OUTPUT);
  digitalWrite(MEGA_256RFR2_RESET, HIGH);
  digitalWrite(MEGA_16U2_RESET, HIGH);
  digitalWrite(DRIVER_FLASH_CS, HIGH);
}

void testJigLoop() {
  if (digitalRead(startButton) == LOW && testIsRunning == false) {
    TD(Serial1.println("Starting test"));
    startTest();
  }
}

void startTest() {
  testIsRunning = true;
  RgbLed.turnOff();
  
  testPowerUSBBattery();
  
  flash16U2();
  flash256RFR2();
  
  if (testFailed == false) {
    testReset();
    testPower3V3();
    testFuelGauge();
    testGPIO();
    
    // testMesh();

    if (testFailed == false) {
      writeEeprom();
    }
  }
  
  if (testFailed == false) {
    RgbLed.green();
  } else {
    RgbLed.red();
  }

  TD(Serial1.println("Test complete"));
  testJigSetup();
}

void testPowerUSBBattery() {
  TD(Serial1.println("- Test Power -"));
  float reading;
  
  TD(Serial1.println("-- Testing all power off"));
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(500);

  reading = readAnalog(VUSB_ADC);
  if (reading > OFF_MAX) {
    TD(Serial1.print("FAIL: VUSB should be low, but it's high: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  reading = readAnalog(VBAT_ADC);
  if (reading > OFF_MAX) {
    TD(Serial1.print("FAIL: VBAT should be low, but it's high: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
 
  TD(Serial1.println("-- Testing VUSB Power"));
  digitalWrite(POWER_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(750);

  reading = readAnalog(VUSB_ADC);
  if (reading < VUSB_MIN) {
    TD(Serial1.print("FAIL: VUSB should be high, but it's low: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  TD(Serial1.println("-- Testing VBAT Power"));
  digitalWrite(POWER_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(VBAT_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(500);
  
  reading = readAnalog(VUSB_ADC);
  if (reading > OFF_MAX) {
    TD(Serial1.print("FAIL: VUSB should be low, but it's high: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  reading = readAnalog(VBAT_ADC);
  if (reading < VBAT_MIN) {
    TD(Serial1.print("FAIL: VBAT should be high, but it's low: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  TD(Serial1.println("-- Testing Power Switch"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(VBAT_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, LOW);
  delay(250);
 
  reading = readAnalog(VUSB_ADC);
  if (reading < VUSB_MIN) {
    TD(Serial1.print("FAIL: VUSB should be high, but it's low: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  reading = readAnalog(VBAT_ADC);
  if (reading < VBAT_MIN) {
    TD(Serial1.print("FAIL: VBAT should be high, but it's low: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }

  return;
}

void testPower3V3() {
  TD(Serial1.println("- Test 3V3 Power -"));
  float reading;
  
  TD(Serial1.println("-- Testing all power off"));
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);
  
  reading = readAnalog(VCC_ADC);
  if (reading > OFF_MAX) {
    TD(Serial1.print("FAIL: 3V3 should be low, but it's high: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  TD(Serial1.println("-- Testing 3V3 via USB Power"));
  digitalWrite(VUSB_SWITCH, HIGH);
  delay(1000);

  reading = readAnalog(VCC_ADC);
  if (reading < VCC_MIN) {
    TD(Serial1.print("FAIL: 3V3 should be high, but it's low: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  TD(Serial1.println("-- Testing 3V3 via VBAT Power"));
  digitalWrite(VBAT_SWITCH, HIGH);
  digitalWrite(VUSB_SWITCH, LOW);
  delay(1000);

  reading = readAnalog(VCC_ADC);
  if (reading < VCC_MIN) {
    TD(Serial1.print("FAIL: 3V3 should be high, but it's low: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }
  
  TD(Serial1.println("-- Testing Power Switch"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(VBAT_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, LOW);
  delay(250);
 
  reading = readAnalog(VCC_ADC);
  if (reading > OFF_MAX) {
    TD(Serial1.print("FAIL: 3V3 should be low, but it's high: "));
    TD(Serial1.println(reading));
    testFailed = true;
  }

  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(250);

  return;
}

void flash16U2() {
  TD(Serial1.println("- Flash 16U2 -"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  resetSPIChipSelectPins();
  delay(500);
  bool err;
  
  AVRProgrammer pgm(MEGA_16U2_RESET, SPI, SPI_CLOCK_DIV32);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  TD(Serial1.println("-- writing bootloader"));
  // if we found a signature, try to write a bootloader (16U2 requires bootloader written first, not sure why!)
  if (pgm.foundSignature() != -1) {
    pgm.eraseChip();
    err = pgm.writeProgram(0x0000, atmega16u2_bootloader, sizeof(atmega16u2_bootloader));
    if (err == true) {
      TD(Serial1.println("FAIL: Verification of writing to 16U2 failed"));
      testFailed = true;
      return;
    }
    TD(Serial1.println("-- writing fuses"));
    pgm.writeFuseBytes(0xEF, 0xD9, 0xF4);
  } else {
    TD(Serial1.println("FAIL: Unable to find signature for 16U2"));
    testFailed = true;
  }

  TD(Serial1.println("-- complete"));
  pgm.end();
  return;
}

void flash256RFR2() {
  TD(Serial1.println("- Flash 256RFR2 -"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  resetSPIChipSelectPins();
  delay(500);
  bool err;
  
  AVRProgrammer pgm(MEGA_256RFR2_RESET, SPI, SPI_CLOCK_DIV64);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  // if we found a signature try to write fuses
  if (pgm.foundSignature() != -1) {
    TD(Serial1.println("-- erasing chip"));
    pgm.eraseChip();
    TD(Serial1.println("-- writing fuses"));
    pgm.writeFuseBytes(0xDE, 0xD0, 0xDE, 0xFF);
    TD(Serial1.println("-- writing bootloader"));
    err = pgm.writeProgram(0x3E000, atmega256rfr2_bootloader, sizeof(atmega256rfr2_bootloader));
    if (err == true) {
      TD(Serial1.println("FAIL: Verification of writing bootloader to 256RFR2 failed"));
      testFailed = true;
      return;
    }
    pgm.writeFuseBytes(0xDE, 0xD0, 0xDE, 0xEF);
  } else {
    TD(Serial1.println("FAIL: Unable to find signature for 256RFR2"));
    testFailed = true;
  }

  pgm.end();
  
  TD(Serial1.println("-- writing sketch"));
  AVRProgrammer pgm2(MEGA_256RFR2_RESET, SPI, SPI_CLOCK_DIV8);
  pgm2.startProgramming();
  pgm2.getSignature();
  
  // if we found a signature try to write the program
  if (pgm2.foundSignature() != -1) {
    err = pgm2.writeProgramFromSerialFlash(0x00000, &DriverFlash, 0x40000, 60031); // be sure not to make this too short! Don't truncate
  }
  
  if (err == true) {
    TD(Serial1.println("FAIL: Verification of writing sketch to 256RFR2 failed"));
    testFailed = true;
    return;
  }
 
  pgm2.end();
  TD(Serial1.println("-- complete"));
  return;
}

void testReset() {
  TD(Serial1.println("- Test Reset -"));
  
  Serial.begin(115200);
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, LOW);
  delay(500);
  
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);
  
  digitalWrite(MEGA_256RFR2_RESET, LOW);
  delay(250);
  //while(Serial.read() != -1);
  digitalWrite(MEGA_256RFR2_RESET, HIGH);
  delay(1000);

  uint32_t time = millis();
  while (!Serial.available()) {
    if (millis() - time > 3000) {
      TD(Serial1.println("FAIL: No serial data received after board reset"));
      testFailed = true;
      return;
    }
  }
  
  char buf[128];
  time = millis();
  int i = 0;
  
  while (millis() - time < 5000 && i < 92) {
    if (Serial.available() > 0) {
      buf[i++] = Serial.read();
      //TD(Serial1.write(buf[i-1]));
    }
  }
  //while(Serial.read() != -1);
  
  //TD(Serial1.println(i));
  
  if (strncmp(buf, "Hello from Pinoccio!", 20) != 0) {
    TD(Serial1.println("FAIL: Incorrect prompt received after board reset"));
    testFailed = true;
    return;
  }
  
  TD(Serial1.println("-- Prompt received after reset"));
  //while(Serial.read() != -1);

  return;
}

void testFuelGauge() {
  TD(Serial1.println("- Test Fuel Gauge -"));
  
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, LOW);
  delay(500);
  Serial.begin(115200);
  while(Serial.read() != -1);
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);

  if (expect(">", 90, 1, 3000) != 0) {
    TD(Serial1.println("FAIL: Incorrect prompt received after board reset"));
    testFailed = true;
    return;
  }
  
  Serial.println("print power.voltage");
  
  uint32_t time = millis();
  char buf[128];
  int i;
  bool done = false;
  int line = 1;
  int voltage;
  
  while (millis() - time < 6000) {
    while (Serial.available()) {
      if (Serial.peek() == '\r' || Serial.peek() == '\n') {    
        if (line == 2) {
          done = true;
          buf[i] = 0;
          break;
        }
        line++;
      }
      buf[i++] = Serial.read();
    }
    if (done == true) {
      break;
    }  
  }
  
  if (done == false) {
    TD(Serial1.println("FAIL: Incorrect prompt received"));
    testFailed = true;
    return;
  }
  
  voltage = atoi(buf);
  
  if (voltage < 360 && voltage > 420) { 
    TD(Serial1.print("FAIL: Fuel gauge outside of range: "));
    TD(Serial1.println(voltage));
    testFailed = true;
    return;
  } else {
    TD(Serial1.println("-- Voltage good "));
  }

  return;
}

// Keep in mind, the logic level converters only work in one direction.  You must set the 
// digital level on the AVR328, and then read it from the Scout under test.  The other way 
// doesn't work.  This only took 12 hours to figure out.  0_o
void testGPIO() {
  TD(Serial1.println("- Test GPIO -"));
 
  
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, LOW);
  delay(500);
  Serial.begin(115200);
  while (Serial.available()) { Serial.read(); }
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);
  
  if (expect(">", 90, 1, 3000) != 0) {
    TD(Serial1.println("FAIL: Incorrect prompt received after board reset"));
    testFailed = true;
    return;
  }
  
  // digital pins
  checkPinVia328(2, 4);
  checkPinVia328(3, 5);
  checkPinVia328(4, 6);
  checkPinVia328(5, 7);
  checkPinVia328(6, 8);
  checkPinVia328(7, 9);
  checkPinVia328(8, 10);
  checkPinVia328(SS, 12);
  
  // analog pins
  checkPinViaDriver(24, 2);   // A0
  checkPinViaDriver(25, 25);  // A1
  checkPinViaDriver(26, 26);  // A2
  checkPinViaDriver(27, 27);  // A3
  checkPinViaDriver(28, 28);  // A4
  checkPinVia328(29, 1);      // A5
  checkPinVia328(30, 2);      // A6
  checkPinVia328(31, 3);      // A7
    
  Serial.end();
  return;
}

void checkPinVia328(int scoutPin, int avr328Pin) {
  int offset = 1;
  int offset2 = 13;
  if (scoutPin >= 10) {
    offset = 2;
    offset2 = 14;
  }
  
  TD(Serial1.print("-- Testing pin "));
  TD(Serial1.println(scoutPin));
  
  Serial.print("pinmode(");
  Serial.print(scoutPin);
  Serial.println(",0)");    
  if (expect(">", 13+offset, 1, 6000) != 0) {
    TD(Serial1.println("FAIL: Incorrect prompt received"));
    testFailed = true;
  }
    
  writeDigital(avr328Pin, 1);
  
  Serial.print("print dr(");
  Serial.print(scoutPin);
  Serial.println(")");
  delay(1);
  if (expect("1", offset2, 1, 6000) != 0) {
    TD(Serial1.print("FAIL: D"));
    TD(Serial1.print(scoutPin));
    TD(Serial1.println(" should be high but it's low"));
    testFailed = true;
  }
  
  writeDigital(avr328Pin, 0);
}

void checkPinViaDriver(int scoutPin, int driverPin) {
  int offset = 1;
  int offset2 = 13;
  if (scoutPin >= 10) {
    offset = 2;
    offset2 = 14;
  }
 
  TD(Serial1.print("-- Testing pin "));
  TD(Serial1.println(scoutPin));
  
  Serial.print("pinmode(");
  Serial.print(scoutPin);
  Serial.println(",0)");    
  if (expect(">", 13+offset, 1, 3000) != 0) {
    TD(Serial1.println("FAIL: Incorrect prompt received"));
    testFailed = true;
  }
  
  pinMode(driverPin, OUTPUT);
  digitalWrite(driverPin, HIGH);

  Serial.print("print dr(");
  Serial.print(scoutPin);
  Serial.println(")");
  delay(1);
  if (expect("1", offset2, 1, 3000) != 0) {
    TD(Serial1.print("FAIL: D"));
    TD(Serial1.print(scoutPin));
    TD(Serial1.println(" should be high but it's low"));
    testFailed = true;
  }
  
  digitalWrite(driverPin, LOW);
  
//  Serial.print("print dr(");
//  Serial.print(scoutPin);
//  Serial.println(")");
//  delay(1);
//  if (expect("0", offset2, 1, 3000) != 0) {
//    TD(Serial1.print("FAIL: D"));
//    TD(Serial1.print(scoutPin));
//    TD(Serial1.println(" should be low but it's high"));
//    testFailed = true;
//  }
}

void testMesh() {
  TD(Serial1.println("- Test Mesh Radio -"));
  
  while(Serial.read() != -1);
  
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, LOW);
  delay(500);
  Serial.begin(115200);
  while (Serial.available()) { Serial.read(); }
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);
  
  if (expect(">", 90, 1, 3000) != 0) {
    TD(Serial1.println("FAIL: Incorrect prompt received after board reset"));
    testFailed = true;
  }
  while (Serial.available()) { Serial.read(); }
  
  TD(Serial1.println("-- Configure mesh settings"));
  
  SYS_Init();
  Scout.meshSetRadio(0x7997);
  //Scout.meshResetSecurityKey();
  
  Serial.println("mesh.config(0)");
  delay(500);
  if (expect(">", 16, 1, 6000) != 0) {
    TD(Serial1.println("FAIL: mesh.config(0) didn't return '>'"));
    testFailed = true;
  }
  
//  Serial.println("mesh.resetkey");
//  delay(500);
//  if (expect(">", 15, 1, 6000) != 0) {
//    TD(Serial1.println("FAIL: mesh.resetkey didn't return '>'"));
//    testFailed = true;
//  }
  
  TD(Serial1.println("-- Testing ping "));
  
  pingScout(0);
  
  uint32_t time = millis();
  while (millis() - time < 2000) {
    Scout.loop();   
    if (pingRSSI != 0) {
      if (pingRSSI > 50) {
        TD(Serial1.print("FAIL: RSSI is lower than -50: "));
        TD(Serial1.println(pingRSSI));
        testFailed = true;
      }
      break;
    } 
  }
  
  if (pingRSSI == 0) {
    TD(Serial1.println("FAIL: No ping response received"));
    testFailed = true;
  }
  
  return;
}

void writeEeprom() {
  
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, LOW);
  delay(500);
  
//  if (RESET_HW_SERIAL == true) {
//    // initialize unique ID;
//    writeHwSerialToFlash(HW_SERIAL_INIT);
//  } 
//  
//  hwSerial = readHwSerialFromFlash();
  TD(Serial1.print("--- Fetched unique ID: 0x"));
  TD(Serial1.println(hwSerial, HEX));
  
  Serial.begin(115200);
  while (Serial.available()) { Serial.read(); }
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
//  digitalWrite(DRIVER_FLASH_CS, HIGH);
//  digitalWrite(MEGA_256RFR2_RESET, HIGH);
//  digitalWrite(MEGA_16U2_RESET, HIGH);
  delay(1000);
  
   
  TD(Serial1.println("-- preparing EEPROM"));
  AVRProgrammer pgm(MEGA_256RFR2_RESET, SPI, SPI_CLOCK_DIV32);
  pgm.startProgramming();
  pgm.getSignature();
  
  // if we found a signature try to write the program
  if (pgm.foundSignature() != -1) {
    byte val[4];   
    
//    TD(Serial1.println("-- erasing EEPROM"));
//    for (int i=0; i<8192; i++) {
//      pgm.writeEeprom(i, 0xFF);
//    }
    
    TD(Serial1.println("-- writing EEPROM"));    
    TD(Serial1.print("--- writing EEPROM version to address 8191: "));
    pgm.writeEeprom(8191, eepromVersion);
    
    if (pgm.readEeprom(8191) != eepromVersion) {
      testFailed = true;
      TD(Serial1.println());
      TD(Serial1.println("FAIL: EEPROM version failed to write to EEPROM"));
    } else {
      TD(Serial1.println(pgm.readEeprom(8191)));
    }
  
    TD(Serial1.print("--- writing hardware version to address 8190: "));
    pgm.writeEeprom(8190, hwVersion);
    if (pgm.readEeprom(8190) != hwVersion) {
      testFailed = true;
      TD(Serial1.println());
      TD(Serial1.println("FAIL: hardware version failed to write to EEPROM"));
    } else {
      TD(Serial1.println(pgm.readEeprom(8190)));
    }
    
    TD(Serial1.print("--- writing hardware family to address 8188-8189: "));
    convertWordToBytes(val, hwFamily);
    pgm.writeEeprom(8188, val[0]);
    pgm.writeEeprom(8189, val[1]);
    if (pgm.readEeprom(8188) != val[0] || pgm.readEeprom(8189) != val[1]) {
      testFailed = true;  
      TD(Serial1.println());
      TD(Serial1.println("FAIL: hardware family failed to write to EEPROM"));
    } else {
      val[0] = pgm.readEeprom(8188);
      val[1] = pgm.readEeprom(8189);
      TD(Serial1.println(convertBytesToWord(val)));
    }
    
    TD(Serial1.print("--- writing unique ID: "));
    convertLongToBytes(val, hwSerial);
    pgm.writeEeprom(8184, val[0]);
    pgm.writeEeprom(8185, val[1]);
    pgm.writeEeprom(8186, val[2]);
    pgm.writeEeprom(8187, val[3]);
    if (pgm.readEeprom(8184) != val[0] || pgm.readEeprom(8185) != val[1] ||
        pgm.readEeprom(8186) != val[2] || pgm.readEeprom(8187) != val[3]) {
      testFailed = true;  
      TD(Serial1.println());
      TD(Serial1.println("FAIL: unique ID failed to write to EEPROM"));
    } else {
      val[0] = pgm.readEeprom(8184);
      val[1] = pgm.readEeprom(8185);
      val[2] = pgm.readEeprom(8186);
      val[3] = pgm.readEeprom(8187);
      TD(Serial1.println(convertBytesToLong(val), HEX));
    }
    
    TD(Serial1.print("--- writing torch color: "));
    pgm.writeEeprom(8127, 0);
    pgm.writeEeprom(8128, 0xFF);
    pgm.writeEeprom(8129, 0);
    if (pgm.readEeprom(8127) != 0 || pgm.readEeprom(8128) != 0xFF || pgm.readEeprom(8129) != 0) {
      testFailed = true;
      TD(Serial1.println());
      TD(Serial1.println("FAIL: Torch color failed to write to EEPROM"));
    } else {
      TD(Serial1.println("Done"));
    }
    
    if (testFailed == false) {
      incrementHwSerial();
    }
  }
  
  pgm.end();
}

void readWire() {
  bool debug = false;
  
  ctr = 0;
  if (Wire.available()) {
    debug ? TD(Serial1.println("Received Wire data")) : false;
  } else {
    debug ? TD(Serial1.println("No response from slave")) : false;
  }
  while (Wire.available()) { 
    wireBuffer[ctr++] = Wire.read();
    debug ? TD(Serial1.write(wireBuffer[ctr-1])) : false;
  }
  debug ? TD(Serial1.println()) : false;
}

numvar i2cSend(void) {
  int len = 2;
  strcpy(cmd, (const char*)getstringarg(1));
  if (strncmp((const char*)cmd, "RA", 2) == 0) {
    len = 5;
  }
  
  return sendCommandToI2C(cmd, len);
}

void sendCommand(const char *cmd, const int responseSize) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  //delay(100);
  Wire.requestFrom(SLAVE_ADDR, responseSize); 
}

int sendCommandToI2C(const char* command, int len) {
  bool debug = false;
  
  wireBuffer[0] = 0;
  sendCommand(command, len);
  
  debug ? TD(Serial1.print("--- Sent: ")) : false;
  for (int i=0; i<strlen(command); i++) {
    debug ? TD(Serial1.write(command[i])) : false;
  }
  debug ? TD(Serial1.println()) : false;
  
  readWire();

  if (strncmp((const char*)wireBuffer, ":", 1) != 0) {
    debug ? TD(Serial1.println("FAIL: Command failed to get a response")) : false;
    testFailed = true;
  } else {   
    debug ? TD(Serial1.print("--- Received: ")) : false;
  }
  
  for (int i=0; i<len; i++) {
    debug ? TD(Serial1.print(wireBuffer[i])) : false;
  }
  debug ? TD(Serial1.println("")) : false;
}

float readAnalog(char *cmd) {
  char result[8];
  sendCommandToI2C(cmd, 5);  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  return ((float) atoi(result) / 1024.0) * 5.0;
}

int readDigital(int pin) {
  char buf[3];
  buf[0] = 0;
  itoa(pin, buf, 10);
  cmd[0] = 0;
  
  if (pin < 10) {
    strcpy(cmd, "RD0");
    strcat(cmd, buf);
  } else {
    strcpy(cmd, "RD");
    strcat(cmd, buf);
  }
  
  char result[8];
  sendCommandToI2C(cmd, 2);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  result[1] = 0;
  return atoi(result);
}

int writeDigital(int pin, int value) {
  char buf[3];
  buf[0] = 0;
  itoa(pin, buf, 10);
  cmd[0] = 0;
  
  if (pin < 10) {
    strcpy(cmd, "WD0");
    strcat(cmd, buf);
  } else {
    strcpy(cmd, "WD");
    strcat(cmd, buf);
  }
  
  buf[0] = 0;
  itoa(value, buf, 10);
  strcat(cmd, buf);
  
  char result[8];
//  TD(Serial1.print("Pin: "));
//  TD(Serial1.println(pin));
//  TD(Serial1.print("Value: "));
//  TD(Serial1.println(value));
//  TD(Serial1.print("Command: "));
//  TD(Serial1.println(cmd));
  sendCommandToI2C(cmd, 2);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  result[1] = 0;
  return atoi(result);
}

int expect(char *expectedString, int start, int length, int timeout) {
  bool debug = false;
  char buf[128];
  char compare[128];
  uint32_t time = millis();
  int i;
  
  debug ? TD(Serial1.print("expectedString: ")) : false;
  debug ? TD(Serial1.println(expectedString)) : false;
  debug ? TD(Serial1.print("start: ")) : false;
  debug ? TD(Serial1.println(start)) : false;
  debug ? TD(Serial1.print("length: ")) : false;
  debug ? TD(Serial1.println(length)) : false;
  debug ? TD(Serial1.print("timeout: ")) : false;
  debug ? TD(Serial1.println(timeout)) : false;
  
  memset(buf, 0, 128);
  memset(compare, 0, 128);
  i = 0;
  
  debug ? TD(Serial1.print("millis: ")) : false;
  debug ? TD(Serial1.println(millis())) : false;
  debug ? TD(Serial1.print("time: ")) : false;
  debug ? TD(Serial1.println(time)) : false;
  
  while (millis() - time < timeout) {
    if (Serial.available() > 0) {
      buf[i] = Serial.read();
      debug ? TD(Serial1.write(buf[i])) : false;
      
      if (i > 0 && buf[i-1] == '>' && buf[i] == ' ') {
        debug ? TD(Serial1.print("Breaking out at ")) : false;
        debug ? TD(Serial1.println(i)) : false;
        //delay(100);
        break;
      }
      
      i++;
    }
  }
  debug ? TD(Serial1.println()) : false;
  
  debug ? TD(Serial1.print("millis: ")) : false;
  debug ? TD(Serial1.println(millis())) : false;
  debug ? TD(Serial1.print("time: ")) : false;
  debug ? TD(Serial1.println(time)) : false;
  
  int ctr = 0;
  for (int j=start; j<start+length; j++) {
//    debug ? TD(Serial1.print("assigning ")) : false;
//    debug ? TD(Serial1.print(buf[j])) : false;
//    debug ? TD(Serial1.print(" to ")) : false;
//    debug ? TD(Serial1.print(compare[ctr])) : false;
    compare[ctr++] = buf[j];
  }
  compare[ctr] = 0;
  debug ? TD(Serial1.println()) : false;
  
  debug ? TD(Serial1.print("compare: ")) : false;
  debug ? TD(Serial1.println(compare)) : false;
  debug ? TD(Serial1.print("expectedString: ")) : false;
  debug ? TD(Serial1.println(expectedString)) : false;
  debug ? TD(Serial1.println("--------------------")) : false;
  
  // flush buffer
  while(Serial.read() != -1);
  
  return strncmp((const char*)compare, expectedString, length);
}

static void pingScout(int address) {
  uint8_t pingCounter = 1;
  pingRSSI = 0;
  
  appDataReq.dstAddr = address;

  appDataReq.dstEndpoint = 1;
  appDataReq.srcEndpoint = 1;
  appDataReq.options = NWK_OPT_ACK_REQUEST|NWK_OPT_ENABLE_SECURITY;
  appDataReq.data = &pingCounter;
  appDataReq.size = sizeof(pingCounter);
  appDataReq.confirm = pingConfirm;
  NWK_DataReq(&appDataReq);

  TD(Serial1.print("Ping "));
  TD(Serial1.print(address));
  TD(Serial1.print(": "));
}

static void pingConfirm(NWK_DataReq_t *req) {
  bool debug = false;
  debug ? TD(Serial1.print("dstAddr: ")) : false;
  debug ? TD(Serial1.println(req->dstAddr, HEX)) : false;
  debug ? TD(Serial1.print("dstEndpoint: ")) : false;
  debug ? TD(Serial1.println(req->dstEndpoint)) : false;
  debug ? TD(Serial1.print("srcEndpoint: ")) : false;
  debug ? TD(Serial1.println(req->srcEndpoint)) : false;
  debug ? TD(Serial1.print("options: ")) : false;
  debug ? TD(Serial1.println(req->options, BIN)) : false;
  debug ? TD(Serial1.print("size: ")) : false;
  debug ? TD(Serial1.println(req->size)) : false;
  debug ? TD(Serial1.print("status: ")) : false;
  debug ? TD(Serial1.println(req->status, HEX)) : false;

  if (req->status == NWK_SUCCESS_STATUS) {
    TD(Serial1.print("1 byte from "));
    TD(Serial1.print(req->dstAddr));
    TD(Serial1.print(" RSSI=-"));
    TD(Serial1.println(req->control));
    pingRSSI = req->control;
  } else {
    TD(Serial1.print("Error: "));
    switch (req->status) {
      case NWK_OUT_OF_MEMORY_STATUS:
        TD(Serial1.print("Out of memory: "));
        break;
      case NWK_NO_ACK_STATUS:
      case NWK_PHY_NO_ACK_STATUS:
        TD(Serial1.print("No acknowledgement received: "));
        break;
      case NWK_NO_ROUTE_STATUS:
        TD(Serial1.print("No route to destination: "));
        break;
      case NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS:
        TD(Serial1.print("Physical channel access failure: "));
        break;
      default:
        TD(Serial1.print("unknown failure: "));
    }
    TD(Serial1.print("("));
    TD(Serial1.print(req->status, HEX));
    TD(Serial1.println(")"));
  }
}

void writeHwSerialToFlash(uint32_t hwSerial) {
  char dataWrite[5];
  char dataCheck[5];
  char dataRead[5];

  digitalWrite(VCC_ENABLE, HIGH); 
  DriverFlash.begin(DRIVER_FLASH_CS, SPI);
  
  TD(Serial1.println("--- Erasing subsector"));
  DriverFlash.subSectorErase(HW_SERIAL_ADDR);
  
  TD(Serial1.print("--- Writing HW unique ID: "));
  convertLongToBytes((byte *)dataWrite, hwSerial);
  TD(Serial1.println(convertBytesToLong((byte *)dataWrite), HEX));
 
  DriverFlash.write(HW_SERIAL_ADDR, &dataWrite, 4);
  TD(Serial1.print("--- Checking previous write: "));
  DriverFlash.read(HW_SERIAL_ADDR, &dataCheck, 4);
  
  TD(Serial1.println(convertBytesToLong((byte *)dataCheck), HEX));
  
  if (strncmp(dataWrite, dataCheck, 4) != 0) {
    TD(Serial1.println("FAIL: Writing serial failed"));
    TD(Serial1.println(atoi(dataWrite), HEX));
    TD(Serial1.println(atoi(dataCheck), HEX));
  } else {
    TD(Serial1.println("--- Write succeeded"));
  }
  DriverFlash.end();
}

uint32_t readHwSerialFromFlash() {
  digitalWrite(VCC_ENABLE, HIGH);
  char dataRead[5];
  DriverFlash.begin(DRIVER_FLASH_CS, SPI);
  DriverFlash.read(HW_SERIAL_ADDR, &dataRead, 4);
  DriverFlash.end();
  return convertBytesToLong((byte *)dataRead);
}

void convertLongToBytes(byte *convBytes, uint32_t target) {
  bool debug = false;
  if (debug) {
    TD(Serial1.print("convertLongToBytes using target: ")); 
    TD(Serial1.println(target, HEX)); 
  }
  
  for (int i=0; i<4; i++) {
    convBytes[i] = (target & 0xFF);
    target = target >> 8;
    if (debug) {
      TD(Serial1.print(convBytes[i], HEX)); 
    }
  }
  if (debug) {
    TD(Serial1.println()); 
  }
  convBytes[4] = 0;
}

uint32_t convertBytesToLong(byte *convBytes) {
  uint32_t target = 0;
  
  for (int i=3; i>=0; i--) {
    target |= convBytes[i];
    if (i > 0) {
      target = target << 8;
    }
  }
  return target;
}

void convertWordToBytes(byte *convBytes, uint16_t target) {
  for (int i=0; i<2; i++) {
    convBytes[i] = (target & 0xFF);
    target = target >> 8;
  }
  convBytes[2] = 0;
}

uint16_t convertBytesToWord(byte *convBytes) {
  uint16_t target = 0;
  
  for (int i=1; i>=0; i--) {
    target |= convBytes[i];
    if (i > 0) {
      target = target << 8;
    }
  }
  return target;
}

void incrementHwSerial() {
  TD(Serial1.println("-- Increment unique ID and store to flash"));
  
  hwSerial = readHwSerialFromFlash() + 1;
  writeHwSerialToFlash(hwSerial);
  TD(Serial1.println("--- Done"));
}
