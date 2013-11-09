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
 - Flash 16U2
   - Read signature
   - Erase chip
   - Write fuses
   - Read fuses
   - Write flash
 
 - Flash 256RFR2
   - Read signature
   - Erase chip
   - Write fuses
   - Read fuses
   - Write EEPROM with incremented ID
   - Read EEPROM
   - Write bootloader flash
   - Write default Scout sketch to flash through USB
 
 - Test reset pin *
   - Assert reset pin
   - Check that new Bitlash header is output
   
 - Test GPIO *
   - Iterate through Bitlash pins as outputs: RX0/TX0, D2-D8, A0-A7, SCK, MOSI, MISO, SS, RX1/TX1, SCL, SDA, BKPKBUS
   - Ensure set correctly
 
 - Test AREF (328p ADC 0-5v)
   - Set AREF to 3.3v, send 3.3v, 1.65v, and 0v to A0-A7, ensure correct reading
 
 - Test RGB LED (using TCS34717FN)
   - Set RGB LED to red, test red is shown
   - Set RGB LED to green, test green is shown
   - Set RGB LED to blue, test blue is shown
 
 - Test power (328p ADC 0-5v)
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

const int SLAVE_ADDR = 2;

FlashClass DriverFlash(SS, SPI);

//serialGLCD lcd;

#define POWERSWITCH_SWITCH 3
#define VBAT_SWITCH 4
#define VUSB_SWITCH 5
#define MEGA_256RFR2_RESET 6
#define MEGA_16U2_RESET 7
const int startButton = 8;

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


void sendCommand(const char *cmd, const int responseSize) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(SLAVE_ADDR, responseSize); 
}


void setup() {
  uint32_t start = millis();
  
  Wire.begin();
  Serial.begin(115200);
  addBitlashFunction("i2c.send", (bitlash_function) i2cSend);
  //Scout.disableShell();
  Scout.setup();
  
  Serial.println("Begin 328 handshake...");
  sendCommand("?", 3);
  readWire();
  if (strncmp((const char*)wireBuffer, "XYZ", 3) != 0) {
    Serial.println("FAIL: Unable to communicate with 328 chip ");
    testFailed = true;
  } else {
    Serial.println("--- 328 chip ready");
  }
  
  while (!DriverFlash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: Serial flash chip not found");
      return;
    }
  }
  Serial.println("--- Serial flash chip found");
   
  //doCommand("i2c.send(\"RA07\")");
  RgbLed.cyan();
  testJigSetup();
}

void loop() {
  Scout.loop();
  testJigLoop();
}

void testJigSetup() {
  
//  doCommand("i2c.send(\"WD000\")");
//  doCommand("i2c.send(\"WD010\")");
//  doCommand("i2c.send(\"WD020\")");
//  doCommand("i2c.send(\"WD030\")");
//  doCommand("i2c.send(\"WD040\")");
//  doCommand("i2c.send(\"WD050\")");
//  doCommand("i2c.send(\"WD060\")");
//  doCommand("i2c.send(\"WD070\")");
//  doCommand("i2c.send(\"WD080\")");
//  doCommand("i2c.send(\"WD090\")");
//  doCommand("i2c.send(\"WD100\")");
//  doCommand("i2c.send(\"WD120\")");
//  doCommand("i2c.send(\"WD170\")");
  
  // disable all switches and chip selects
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  pinMode(POWERSWITCH_SWITCH, OUTPUT);
  digitalWrite(POWERSWITCH_SWITCH, LOW);
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

  pinMode(startButton, INPUT);
  digitalWrite(startButton, HIGH);
  
  testIsRunning = false;
  testFailed = false;
  
  Serial.println("Scout Test Jig ready to go!");
  //lcd.clearLCD();
  //Serial1.println("Hello");
}

void testJigLoop() {
  if (digitalRead(startButton) == LOW && testIsRunning == false) {
    Serial.println("Starting test");
    startTest();
  }
}

void startTest() {
  testIsRunning = true;
  RgbLed.turnOff();
  
  //testPower();  
    
  flash16U2();
  flash256RFR2();

  //testReset();
  //testGPIO();
  //testAREF();
  
  //testRGBLED();
  
  //testMesh();

  if (testFailed == false) {
    RgbLed.green();
  } 
  else {
    RgbLed.red();
  }

  Serial.println("Test complete");
  testJigSetup();
}

void testPower() {
  Serial.println("- Test Power -");
  char result[32];
  
  Serial.println("-- Testing USB Power");
  
  digitalWrite(VUSB_SWITCH, LOW);
  delay(500);

  sendCommandToI2C("RA0", 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  if (atoi(result) > 512) {
    Serial.println("FAIL: VUSB should be low, but it's high");
    testFailed = true;
  }
  
  sendCommandToI2C("RA1", 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  if (atoi(result) > 512) {
    Serial.println("FAIL: VBAT should be low, but it's high");
    testFailed = true;
  }
  
  sendCommandToI2C("RA2", 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  if (atoi(result) > 512) {
    Serial.println("FAIL: VCC should be low, but it's high");
    testFailed = true;
  }
  
  digitalWrite(VUSB_SWITCH, HIGH);
  delay(500);
  
  sendCommandToI2C("RA0", 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  if (atoi(result) < 1000) {
    Serial.println("FAIL: VUSB should be high, but it's low");
    testFailed = true;
  }
  
  sendCommandToI2C("RA1", 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  if (atoi(result) < 1000) {
    Serial.println("FAIL: VBAT should be high, but it's low");
    testFailed = true;
  }
  
  sendCommandToI2C("RA2", 5);
  strncpy(result, wireBuffer + 1, strlen(wireBuffer));
  if (atoi(result) < 1000) {
    Serial.println("FAIL: VCC should be high, but it's low");
    testFailed = true;
  }
  
  return;
}

void flash16U2() {
  Serial.println("- Flash 16U2 -");
  digitalWrite(VUSB_SWITCH, HIGH);
  delay(500);
  
  AVRProgrammer pgm = AVRProgrammer(MEGA_16U2_RESET, SPI_CLOCK_DIV32);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  Serial.println("-- writing bootloader");
  // if we found a signature, try to write a bootloader (16U2 requires bootloader written first, not sure why!)
  if (pgm.foundSignature() != -1) {
    pgm.eraseChip();
    pgm.writeProgram(0x0000, atmega16u2_bootloader, sizeof(atmega16u2_bootloader));
    pgm.writeFuseBytes(0xEF, 0xD9, 0xF4);
  } else {
    testFailed = true;
  }

  //pgm.readProgram();
  Serial.println("-- complete");
  pgm.end();
  return;
}

void flash256RFR2() {
  Serial.println("- Flash 256RFR2 -");

  AVRProgrammer pgm = AVRProgrammer(MEGA_256RFR2_RESET, SPI_CLOCK_DIV64);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();

  //pgm.readProgram(0x0000, 100);
  //return;
  
  Serial.println("-- writing fuses");
  // if we found a signature try to write fuses
  if (pgm.foundSignature() != -1) {
    pgm.eraseChip();
    pgm.writeFuseBytes(0xFF, 0xD0, 0xFE);
    pgm.writeProgram(0x3E000, atmega256rfr2_bootloader, sizeof(atmega256rfr2_bootloader));
  } else {
    testFailed = true;
  }

  pgm.end();
  
  Serial.println("-- writing sketch");
  pgm = AVRProgrammer(MEGA_256RFR2_RESET, SPI_CLOCK_DIV8);
  pgm.startProgramming();
  pgm.getSignature();
  
  // if we found a signature try to write the program
  if (pgm.foundSignature() != -1) {
    pgm.writeProgramFromSerialFlash(0x00000, &DriverFlash, 0x40000, 51000); // be sure not to make this too short! Don't truncate
    pgm.writeFuseBytes(0xFF, 0xD0, 0xFE);
  }
  
  
  pgm.end();
  Serial.println("-- complete");
  return;
}

void testReset() {
  Serial.println("- Test Reset -");

  return;
}

void testGPIO() {
  Serial.println("- Test GPIO -");
  
  return;
}

void testAREF() {
  Serial.println("- Test AREF -");

  return;
}

void testRGBLED() {
  Serial.println("- Test RGB LED -");

  return;
}

void testMesh() {
  Serial.println("- Test Mesh Radio -");

  return;
}

void readWire() {
  ctr = 0;
  if (Wire.available()) {
    //Serial.println("Received Wire data");
  } else {
    Serial.println("No response from slave");
  }
  while (Wire.available()) { 
    wireBuffer[ctr++] = Wire.read();
    //Serial.write(wireBuffer[ctr-1]);
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

int sendCommandToI2C(const char* command, int len) {
  wireBuffer[0] = 0;
  sendCommand(command, len);
  readWire();
  if (strncmp((const char*)wireBuffer, ":", 1) != 0) {
    Serial.println("FAIL: Command failed to get a response");
    testFailed = true;
  } else {
    Serial.print("--- ");
    Serial.print(" ");
    Serial.print(command[0]);
    Serial.print(command[1]);
    Serial.print(command[2]);
    Serial.print(command[3]);
    
    for (int i=0; i<len; i++) {
      Serial.print(wireBuffer[i]);
    }
    Serial.println("");
  }
}


