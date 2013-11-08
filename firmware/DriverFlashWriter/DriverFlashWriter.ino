#include <LeadScout.h>
#include <SPI.h>
#include <Wire.h>

FlashClass DriverFlash(SS, SPI);
uint32_t start = millis();

uint32_t addr = 0x40000;
int ctr = 0;
byte dataRead[64];
byte dataWritten[64];
  
#define MEGA_256RFR2_RESET 6
#define MEGA_16U2_RESET 7

#define BUFSIZE 32

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
  
  // Store flash hex in sector 4 (0x40000 - 0x4F000, 61,440 bytes)
  Serial.println("--- Erasing sector 0x40000");
  DriverFlash.sectorErase(addr); 
  Serial.println("--- Ready for hex bytes");
}

void loop() {
  
  while (Serial.available()) {
//    Serial.print(Serial.read());
//    Serial.print(Serial.peek());
//    Serial.print(":");
    dataRead[ctr++] = (byte)Serial.read();
//    Serial.println(dataRead[ctr-1], DEC);
    
    if (ctr == BUFSIZE) {
      dataRead[BUFSIZE] = 0;

      DriverFlash.write(addr, &dataRead, BUFSIZE);
      DriverFlash.read(addr, &dataWritten, BUFSIZE);
     
      if (strncmp((const char*)dataRead, (const char*)dataWritten, BUFSIZE) != 0) {
        Serial.print("FAIL: Data failed to write to and read from flash at address: ");
        Serial.println(addr, HEX);
        
        dataRead[ctr] = 0;
        dataWritten[ctr] = 0;
      
        Serial.print("dataRead: ");
        for (int i=0; i<BUFSIZE; i++) {
          Serial.print(dataRead[i], HEX);
        }
        Serial.println();
        Serial.print("dataWritten: ");
        for (int i=0; i<BUFSIZE; i++) {
          Serial.print(dataWritten[i], HEX);
        }
        Serial.println();
      } else {
        Serial.println("OK");
      }
     
      ctr = 0;
      addr += BUFSIZE;
    } 
  }
}

