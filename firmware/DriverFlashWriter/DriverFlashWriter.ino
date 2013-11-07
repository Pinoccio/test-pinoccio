#include <LeadScout.h>
#include <SPI.h>
#include <Wire.h>

FlashClass DriverFlash(SS, SPI);
uint32_t start = millis();

uint32_t addr = 0;
int ctr = 0;
byte dataRead[64];
byte dataWritten[64];
  
#define MEGA_256RFR2_RESET 6
#define MEGA_16U2_RESET 7

#define BUFSIZE 32

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  
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
  
  //Serial.println("--- Erasing serial flash chip");
  //DriverFlash.bulkErase();
  
  Serial.println("--- Ready for hex bytes");
//  
//  for (uint32_t i=0; i<16777216; i+=16) {
//    if (i % 1024 == 0) {
//      Serial.print("Testing address: ");
//      Serial.println(i, HEX);
//    }
//    
//    memset(dataFromChip, ' ', 16);
//    dataFromChip[15] = 0;
//    
//    // Write some data to RAM
//    Flash.write(i, dataToChip, 16);
//    //delay(100);
//  
//    // Read it back to a different buffer
//    Flash.read(i, dataFromChip, 16);
//    
//    // Write it to the serial port
//    if (strcmp((const char*)dataToChip, (const char*)dataFromChip) != 0) {
//      Serial.print("FAIL: Data failed to write to and read from flash at address: ");
//      Serial.println(i);
//      Serial.println(dataToChip);
//      Serial.println(dataFromChip);
//    }
//    
//    Flash.sectorErase(i);  
//  }
//  
//  Serial.println("Driver flash chip checks out fine");
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
      //strcpy(dataWritten, dataRead);
      
      //Serial.print("Got bytes, would write the following to addy 0x");
      //Serial.println(addr, HEX);
      
      //for (int i=0; i<BUFSIZE; i++) {
      //  Serial.print(dataRead[i], DEC);
      //}
      //Serial.println();
      
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
 
//    Serial.print("got ");
//    Serial.print(dataIn);
//    Serial.print(" (0x");
//    Serial.print(dataIn, HEX);
//    Serial.println(")");
//    Serial.print("wrote ");
//    Serial.println(dataOut, HEX);
//    if (dataIn != dataOut) {
//      Serial.println("write failed :(");
//    }
  }
  
//  if (millis() - start > 1000 && !done) {
//    Serial.println("First 100 bytes of written flash:");
//    for (addr=0; addr<10; addr++) {
//      DriverFlash.read(addr++, &data, 1);
//      Serial.print(data, HEX);
//      Serial.println("");
//    } 
//    done = true;
//  }
}

