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
 
 - Test AREF (328p ADC 0-5v)
   - Set AREF to 3.3v, send 3.3v, 1.65v, and 0v to A0-A7, ensure correct reading
 
 - Test RGB LED (using TCS34717FN)
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
#include <LeadScout.h>

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

const float VUSB_MIN = 4.8;
const float VBAT_MIN = 3.7;
const float VCC_MIN = 3.2;
const float OFF_MAX = 0.8;

#define DRIVER_FLASH_CS SS
#define AREF_SWITCH A0
#define JOY_V A5
#define JOY_H A6
#define JOY_SWITCH A7

bool testIsRunning;
bool testFailed;

char wireBuffer[256];
char* cmd = (char*)malloc(32);
int ctr = 0;

int foundSig = -1;

void setup() {
  uint32_t start = millis();
  
  Wire.begin();
  Serial.begin(115200);
  //Serial1.begin(115200);
  addBitlashFunction("i2c.send", (bitlash_function) i2cSend);
  //Scout.disableShell();
  Scout.setup();
  
  TD(Serial.println("- Initialize Test Jig"));
  sendCommand("?", 3);
  readWire();
  if (strncmp((const char*)wireBuffer, DRIVER_VERSION, 3) != 0) {
    TD(Serial.println("FAIL: Unable to communicate with 328 chip "));
    testFailed = true;
  } else {
    TD(Serial.println("-- 328 chip ready"));
  }
  
  bool flashFound = false;
  while (millis() - start < 1000) {
    if (DriverFlash.available()) {
      TD(Serial.println("-- Serial flash chip found"));
      flashFound = true;
      break;
    }
  }
  
  if (!flashFound) {
    TD(Serial.println("FAIL: Serial flash chip not found"));
  }

  RgbLed.cyan();
  testJigSetup();
}

void loop() {
  Scout.loop();
  testJigLoop();
}

void testJigSetup() {
  
  // disable all switches and chip selects
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  pinMode(POWER_SWITCH, OUTPUT);
  digitalWrite(POWER_SWITCH, LOW);
  pinMode(VBAT_SWITCH, OUTPUT);
  digitalWrite(VBAT_SWITCH, LOW);
  pinMode(VUSB_SWITCH, OUTPUT);
  digitalWrite(VUSB_SWITCH, LOW);
  pinMode(MEGA_256RFR2_RESET, OUTPUT);
  digitalWrite(MEGA_256RFR2_RESET, HIGH);
  pinMode(MEGA_16U2_RESET, OUTPUT);
  digitalWrite(MEGA_16U2_RESET, HIGH);
  pinMode(DRIVER_FLASH_CS, OUTPUT);
  digitalWrite(DRIVER_FLASH_CS, HIGH);
  
  pinMode(BACKPACK_BUS, INPUT);
  digitalWrite(BACKPACK_BUS, LOW);
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
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
  
  testIsRunning = false;
  testFailed = false;
  
  TD(Serial.println("--- Scout Test Jig ready to go! ---"));
  //lcd.clearLCD();
  //Serial1.println("Hello");
}

void testJigLoop() {
  if (digitalRead(startButton) == LOW && testIsRunning == false) {
    TD(Serial.println("Starting test"));
    startTest();
  }
}

void startTest() {
  testIsRunning = true;
  RgbLed.turnOff();
  
  //testPower();  
  
  //flash16U2();
  //flash256RFR2();
  
  //testReset();
  testGPIO();
  //testAREF();
  
  //testRGBLED();
  
  //testMesh();

  if (testFailed == false) {
    RgbLed.green();
  } else {
    RgbLed.red();
  }

  TD(Serial.println("Test complete"));
  testJigSetup();
}

void testPower() {
  TD(Serial.println("- Test Power -"));
  float reading;
  
  TD(Serial.println("-- Testing all power off"));
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(500);

  if (reading = readAnalog(VUSB_ADC) > OFF_MAX) {
    TD(Serial.print("FAIL: VUSB should be low, but it's high: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  if (reading = readAnalog(VBAT_ADC) > OFF_MAX) {
    TD(Serial.print("FAIL: VBAT should be low, but it's high: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  if (reading = readAnalog(VCC_ADC) > OFF_MAX) {
    TD(Serial.print("FAIL: VCC should be low, but it's high: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  TD(Serial.println("-- Testing VUSB Power"));
  digitalWrite(POWER_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(800);

  if (reading = readAnalog(VUSB_ADC) < VUSB_MIN) {
    TD(Serial.print("FAIL: VUSB should be high, but it's low: "));
    TD(Serial.println(reading));
    testFailed = true;
  }

  if (reading = readAnalog(VCC_ADC) < VCC_MIN) {
    TD(Serial.print("FAIL: VCC should be high, but it's low: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  TD(Serial.println("-- Testing VBAT Power"));
  digitalWrite(POWER_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(VBAT_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);
  
  if (reading = readAnalog(VUSB_ADC) > OFF_MAX) {
    TD(Serial.print("FAIL: VUSB should be low, but it's high: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  if (reading = readAnalog(VBAT_ADC) < VBAT_MIN) {
    TD(Serial.print("FAIL: VBAT should be high, but it's low: "));
    TD(Serial.println(reading));
    testFailed = true;
  }

  if (reading = readAnalog(VCC_ADC) < VCC_MIN) {
    TD(Serial.print("FAIL: VCC should be high, but it's low: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  TD(Serial.println("-- Testing Power Switch"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(VBAT_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, LOW);
  delay(800);
 
  if (reading = readAnalog(VUSB_ADC) < VUSB_MIN) {
    TD(Serial.print("FAIL: VUSB should be high, but it's low: "));
    TD(Serial.println(reading));
    testFailed = true;
  }
  
  if (reading = readAnalog(VBAT_ADC) < VBAT_MIN) {
    TD(Serial.print("FAIL: VBAT should be high, but it's low: "));
    TD(Serial.println(reading));
    testFailed = true;
  }

  if (reading = readAnalog(VCC_ADC) > OFF_MAX) {
    TD(Serial.print("FAIL: VCC should be low, but it's high: "));
    TD(Serial.println(reading));
    testFailed = true;
  }

  return;
}

void flash16U2() {
  TD(Serial.println("- Flash 16U2 -"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(500);
  
  AVRProgrammer pgm = AVRProgrammer(MEGA_16U2_RESET, SPI_CLOCK_DIV32);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  TD(Serial.println("-- writing bootloader"));
  // if we found a signature, try to write a bootloader (16U2 requires bootloader written first, not sure why!)
  if (pgm.foundSignature() != -1) {
    pgm.eraseChip();
    pgm.writeProgram(0x0000, atmega16u2_bootloader, sizeof(atmega16u2_bootloader));
    TD(Serial.println("-- writing fuses"));
    pgm.writeFuseBytes(0xEF, 0xD9, 0xF4);
  } else {
    testFailed = true;
  }

  TD(Serial.println("-- complete"));
  pgm.end();
  return;
}

void flash256RFR2() {
  TD(Serial.println("- Flash 256RFR2 -"));
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(500);

  AVRProgrammer pgm = AVRProgrammer(MEGA_256RFR2_RESET, SPI_CLOCK_DIV64);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  TD(Serial.println("-- writing fuses"));
  // if we found a signature try to write fuses
  if (pgm.foundSignature() != -1) {
    pgm.eraseChip();
    pgm.writeFuseBytes(0xFF, 0xD0, 0xFE);
    TD(Serial.println("-- writing bootloader"));
    pgm.writeProgram(0x3E000, atmega256rfr2_bootloader, sizeof(atmega256rfr2_bootloader));
  } else {
    testFailed = true;
  }

  pgm.end();
  
  TD(Serial.println("-- writing sketch"));
  pgm = AVRProgrammer(MEGA_256RFR2_RESET, SPI_CLOCK_DIV8);
  pgm.startProgramming();
  pgm.getSignature();
  
  // if we found a signature try to write the program
  if (pgm.foundSignature() != -1) {
    pgm.writeProgramFromSerialFlash(0x00000, &DriverFlash, 0x40000, 51000); // be sure not to make this too short! Don't truncate
    pgm.writeFuseBytes(0xFF, 0xD0, 0xFE);
  }
 
  pgm.end();
  TD(Serial.println("-- complete"));
  return;
}

void testReset() {
  TD(Serial.println("- Test Reset -"));
  uint32_t time = millis();
  
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, LOW);
  digitalWrite(POWER_SWITCH, LOW);
  delay(1000);
  
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(500);

  digitalWrite(MEGA_256RFR2_RESET, LOW);
  while(Serial.read() != -1);
  delay(250);
  digitalWrite(MEGA_256RFR2_RESET, HIGH);
  delay(1000);
  
  while (!Serial.available()) {
    if (millis() - time > 5000) {
      TD(Serial.println("FAIL: No serial data received after board reset"));
      testFailed = true;
      return;
    }
  }
  
  char buf[20];
  time = millis();
  int i = 0;
  
  while (millis() - time < 5000 && i < 20) {
    if (Serial.available()) {
      TD(Serial.write(Serial.peek()));
      buf[i++] = Serial.read();
    }
  }
  
  if (strncmp(buf, "Hello from Pinoccio!", 20) != 0) {
    TD(Serial.println("FAIL: Incorrect prompt received after board reset"));
    testFailed = true;
    return;
  }
  
  TD(Serial.println("-- Prompt received after reset"));
  return;
}

void testGPIO() {
  TD(Serial.println("- Test GPIO -"));
  digitalWrite(VBAT_SWITCH, LOW);
  digitalWrite(VUSB_SWITCH, HIGH);
  digitalWrite(POWER_SWITCH, HIGH);
  delay(1000);
  
  char cmd[8] = "";
  char buf[4];
  
  for (int i=2; i<=14; i++) {
    delay(100);
    
    TD(Serial.print("--- Setting D"));
    TD(Serial.print(i));
    TD(Serial.println(" to 1"));
    Serial.print("pinmode(");
    Serial.print(i);
    Serial.println(",1)");
    Serial.print("dw(");
    Serial.print(i);
    Serial.println(",1)");
    delay(5000);
    
    itoa(i, buf, 10);
    cmd[0] = 0;
  
    if (i < 10) {
      strcpy(cmd, "RD0");
      strcat(cmd, buf);
    } else {
      strcpy(cmd, "RD");
      strcat(cmd, buf);
    }
    
    if (readDigital(cmd) != 1) {
      TD(Serial.print("FAIL: D"));
      TD(Serial.print(i));
      TD(Serial.println(" should be high but it's low"));
      testFailed = true;
    }
    
    TD(Serial.print("--- Setting D"));
    TD(Serial.print(i));
    TD(Serial.println(" to 0"));
    Serial.print("dw(");
    Serial.print(i);
    Serial.println(",0)");
    delay(5000);
 
    if (readDigital(cmd) != 0) {
      TD(Serial.print("FAIL: D"));
      TD(Serial.print(i));
      TD(Serial.println(" should be low but it's high"));
      testFailed = true;
    }
    
    return;
    cmd[0] = 0;
  }
  
  // backpack bus
  TD(Serial.println("--- Setting D19 to 1"));
  Serial.println("pinmode(19,1)");
  Serial.print("dw(19,1)");
  
  itoa(19, buf, 10);
  cmd[0] = 0;
  strcpy(cmd, "RD");
  strcat(cmd, buf);
  
  if (readDigital(cmd) != 1) {
    TD(Serial.println("FAIL: D19 should be high but it's low"));
    testFailed = true;
  }
  
  TD(Serial.println("--- Setting D19 to 0"));
  Serial.print("dw(19,0)");
  
  if (readDigital(cmd) != 0) {
    TD(Serial.println("FAIL: D19 should be low but it's high"));
    testFailed = true;
  }
  
  cmd[0] = 0;
  return;
}

void testAREF() {
  TD(Serial.println("- Test AREF -"));

  return;
}

void testRGBLED() {
  TD(Serial.println("- Test RGB LED -"));

  return;
}

void testMesh() {
  TD(Serial.println("- Test Mesh Radio -"));

  return;
}

void readWire() {
  bool debug = false;
  
  ctr = 0;
  if (Wire.available()) {
    debug ? TD(Serial.println("Received Wire data")) : false;
  } else {
    debug ? TD(Serial.println("No response from slave")) : false;
  }
  while (Wire.available()) { 
    wireBuffer[ctr++] = Wire.read();
    debug ? TD(Serial.write(wireBuffer[ctr-1])) : false;
  }   
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
  delay(100);
  Wire.requestFrom(SLAVE_ADDR, responseSize); 
}

int sendCommandToI2C(const char* command, int len) {
  bool debug = true;
  
  wireBuffer[0] = 0;
  sendCommand(command, len);
  
  debug ? TD(Serial.print("--- Sent: ")) : false;
  for (int i=0; i<strlen(command); i++) {
    debug ? TD(Serial.write(command[i])) : false;
  }
  debug ? TD(Serial.println()) : false;
  
  readWire();

  if (strncmp((const char*)wireBuffer, ":", 1) != 0) {
    debug ? TD(Serial.println("FAIL: Command failed to get a response")) : false;
    testFailed = true;
  } else {   
    debug ? TD(Serial.print("--- Received: ")) : false;
  }
  
  for (int i=0; i<len; i++) {
    debug ? TD(Serial.print(wireBuffer[i])) : false;
  }
  debug ? TD(Serial.println("")) : false;
}

float readAnalog(char *cmd) {
  char result[8];
  sendCommandToI2C(cmd, 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  return ((float) atoi(result) / 1024.0) * 5.0;
}

int readDigital(char *cmd) {
  char result[8];
  sendCommandToI2C(cmd, 2);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  result[1] = 0;
  return atoi(result);
}
