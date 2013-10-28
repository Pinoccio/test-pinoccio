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


//#include <serialGLCDlib.h>
#include <Scout.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/pgmspace.h>

const int SLAVE_ADDR = 2;

//serialGLCD lcd(Serial1);

#define POWERSWITCH_SWITCH 3
// BROKEN: #define VBAT_SWITCH 4
#define VUSB_SWITCH 5
#define MEGA_256RFR2_RESET 6
#define MEGA_16U2_RESET 7
const int startButton = 8;

#define DRIVER_FLASH_CS SS
#define AREF_SWITCH A0
#define JOY_V A5
#define JOY_H A6
#define JOY_SWITCH A7

bool testIsRunning = false;
bool testFailed = false;

char wireBuffer[256];
char* cmd = (char*)malloc(32);
int ctr = 0;


void sendCommand(const char *cmd, const int responseSize) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(SLAVE_ADDR, responseSize); 
}


void setup() {
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
  //doCommand("i2c.send(\"RD08\")");
  
  
  testJigSetup();
}

void loop() {
  Scout.loop();
  testJigLoop();
}

void readWire() {
  ctr = 0;
  if (Wire.available()) {
    Serial.println("Received Wire data");
  } else {
    Serial.println("No response from slave");
  }
  while (Wire.available()) { 
    wireBuffer[ctr++] = Wire.read();
    //Serial.write(wireBuffer[ctr-1]);
  }   
}

void testJigSetup() {
  
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
  
  // disable all switches and chip selects 
  pinMode(POWERSWITCH_SWITCH, OUTPUT);
  digitalWrite(POWERSWITCH_SWITCH, LOW);
  //pinMode(VBAT_SWITCH, OUTPUT);
  //digitalWrite(VBAT_SWITCH, LOW);
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
  
  Serial.println("Scout Test Jig ready to go!");
  RgbLed.cyan();
  
  //lcd.clearLCD();
}

void testJigLoop() {
  if (digitalRead(startButton) == LOW) {
    Serial.println("Starting test");
    startTest();
  }
}

void startTest() {
  RgbLed.turnOff();
  if (testIsRunning == true) {
    Serial.println("Test is already running");
    return;
  }
  testIsRunning = true;
  testFailed = false;

  testPower();  
    
  //flash16U2();
  //flash256RFR2();

  //testReset();
  //testGPIO();
  //testAREF();
  
  //testRGBLED();
  
  //testMesh();

  testIsRunning = false;

  if (testFailed == false) {
    RgbLed.green();
  } 
  else {
    RgbLed.red();
  }
  Serial.println("Test complete");
}

void testPower() {
  Serial.println("- Test Power -");

  Serial.println("-- Testing USB Power");
  digitalWrite(VUSB_SWITCH, HIGH);
  delay(5000);
  digitalWrite(VUSB_SWITCH, LOW);
  //Serial.println("-- Testing VBAT Power");
  //digitalWrite(VBAT_SWITCH, HIGH);
  //delay(5000);
  //digitalWrite(VBAT_SWITCH, LOW);
  Serial.println("-- Testing Power Switch");
  digitalWrite(POWERSWITCH_SWITCH, HIGH);
  delay(5000);
  digitalWrite(POWERSWITCH_SWITCH, LOW);
  
  return;
}

void flash16U2() {
  Serial.println("- Flash 16U2 -");

//  digitalWrite(RESET, HIGH);
//  SPI.begin();
//
//  // slow down SPI for benefit of slower processors like the Attiny
//  SPI.setClockDivider(SPI_CLOCK_DIV8);
//
//  pinMode(SCK, OUTPUT);
//  RESET = MEGA_16U2_RESET;
//  
//  // set up Timer 1
//  TCCR1A = _BV (COM1A0);  // toggle OC1A on Compare Match
//  TCCR1B = _BV(WGM12) | _BV(CS10);   // CTC, no prescaling
//  OCR1A =  0;       // output every cycle
//
//  startProgramming();
//  getSignature();
//  getFuseBytes();
//
//  // if we found a signature try to write a bootloader
//  if (foundSig != -1) {
//    writeBootloader();
//  }
//  //readProgram ();
//
//  // release reset
//  digitalWrite (RESET, HIGH);
  
  return;
}

void flash256RFR2() {
  Serial.println("- Flash 256RFR2 -");

//  digitalWrite(RESET, HIGH);
//  SPI.begin();
//
//  // slow down SPI for benefit of slower processors like the Attiny
//  SPI.setClockDivider(SPI_CLOCK_DIV8);
//
//  pinMode(SCK, OUTPUT);
//  RESET = MEGA_256RFR2_RESET;
//  
//  // set up Timer 1
//  TCCR1A = _BV (COM1A0);  // toggle OC1A on Compare Match
//  TCCR1B = _BV(WGM12) | _BV(CS10);   // CTC, no prescaling
//  OCR1A =  0;       // output every cycle
//
//  startProgramming();
//  getSignature();
//  getFuseBytes();
//
//  // if we found a signature try to write a bootloader
//  if (foundSig != -1) {
//    writeBootloader();
//  }
//  //readProgram ();
//
//  // release reset
//  digitalWrite (RESET, HIGH);
  
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

numvar i2cSend(void) {
  int len = 2;
  strcpy(cmd, (const char*)getstringarg(1));
  if (strncmp((const char*)cmd, "RA", 2) == 0) {
    len = 4;
  }
  
  sendCommand(cmd, len);
  readWire();
  if (strncmp((const char*)wireBuffer, ":", 1) != 0) {
    Serial.println("FAIL: Command failed to get a response");
    testFailed = true;
  } else {
    Serial.print("--- ");
    Serial.print(" ");
    Serial.print(cmd[0]);
    Serial.print(cmd[1]);
    Serial.print(cmd[2]);
    Serial.print(cmd[3]);
    
    for (int i=0; i<len; i++) {
      Serial.print(wireBuffer[i]);
    }
    Serial.println("");
  }
}
