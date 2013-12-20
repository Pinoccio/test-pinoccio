#include <LeadScout.h>
#include <SPI.h>
#include <Wire.h>

FlashClass DriverFlash(SS, SPI);
  
#define MEGA_256RFR2_RESET 6
#define MEGA_16U2_RESET 7
#define BUFSIZE 16384

uint32_t start = millis();
uint32_t addr = 0x40000;
int ctr = 0;
byte dataRead[BUFSIZE];


void readData(uint32_t address, uint32_t length) {
  Serial.println();
  Serial.print("Reading ");
  Serial.print(length);
  Serial.print(" bytes of program memory starting at address 0x");
  Serial.println(address, HEX);
  
  int ctr = 0;
  
  for (uint32_t i = addr; i <= addr + length; i++) {
    
    if (i % BUFSIZE == 0) {
      ctr = 0;
      DriverFlash.read(i, &dataRead, BUFSIZE);
    }
   
    if (i % 16 == 0) {
      Serial.print(address + i, HEX);
      Serial.print(": ");
    }
    
    Serial.print("0x");
    Serial.print(dataRead[ctr++], HEX);
    Serial.print(" ");
    
    if (i % 16 == 15) {
      Serial.println();
    }
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  
  // Ensure 3V3 is high, otherwise driver flash chip isn't powered
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  pinMode(MEGA_256RFR2_RESET, OUTPUT);
  digitalWrite(MEGA_256RFR2_RESET, HIGH);
  pinMode(MEGA_16U2_RESET, OUTPUT);
  digitalWrite(MEGA_16U2_RESET, HIGH);
  
  while (!DriverFlash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: Serial flash chip not found");
      return;
    }
  }
  Serial.println("--- Serial flash chip found");
  
  //DriverFlash.bulkErase(); 
  // Store flash hex in sector 4 (0x40000 - 0x4F000, 61,440 bytes)
  Serial.println("--- Reading sector 0x40000");
  readData(addr, 69000);
}

void loop() { }
