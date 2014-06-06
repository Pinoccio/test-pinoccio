#include <SPI.h>
#include <Wire.h>
#include <Scout.h>
#include <GS.h>
#include <bitlash.h>
#include <lwm.h>
#include <js0n.h>
#include <peripherals/Flash.h>

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
  
  DriverFlash.begin(SS, SPI);

  bool flashFound = false;
  while (millis() - start < 1000) {
    if (DriverFlash.available()) {
      Serial.println("-- Serial flash chip found");
      flashFound = true;
      DriverFlash.end();
      break;
    }
  }
  
  if (!flashFound) {
    Serial.println("FAIL: Serial flash chip not found");
    return;
  }
  
  DriverFlash.begin(SS, SPI);
  //DriverFlash.bulkErase(); 
  // Store flash hex in sector 4 (0x40000)
  Serial.println("--- Erasing sector 0x40000");
  DriverFlash.sectorErase(addr);
  Serial.println("--- Erasing sector 0x50000");
  DriverFlash.sectorErase(addr+0x10000);
  Serial.println("--- Erasing sector 0x60000");
  DriverFlash.sectorErase(addr+0x20000);
  Serial.println("--- Erasing sector 0x70000");
  DriverFlash.sectorErase(addr+0x30000); 
  Serial.println("--- Ready for hex bytes");
  DriverFlash.end();
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

      DriverFlash.begin(SS, SPI);
      DriverFlash.write(addr, &dataRead, BUFSIZE);
      DriverFlash.read(addr, &dataWritten, BUFSIZE);
      DriverFlash.end();
      
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

