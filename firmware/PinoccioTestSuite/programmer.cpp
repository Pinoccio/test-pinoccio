// Atmega chip programmer
// Author: Nick Gammon
// Date: 22nd May 2012
// Version: 1.19

// Version 1.1: Reset foundSig to -1 each time around the loop.
// Version 1.2: Put hex bootloader data into separate files
// Version 1.3: Added verify, and MD5 sums
// Version 1.4: Added signatures for ATmeag8U2/16U2/32U2 (7 May 2012)
// Version 1.5: Added signature for ATmega1284P (8 May 2012)
// Version 1.6: Allow sketches to read bootloader area (lockbyte: 0x2F)
// Version 1.7: Added choice of bootloaders for the Atmega328P (8 MHz or 16 MHz)
// Version 1.8: Output an 8 MHz clock on pin 9
// Version 1.9: Added support for Atmega1284P, and fixed some bugs
// Version 1.10: Corrected flash size for Atmega1284P.
// Version 1.11: Added support for Atmega1280. Removed MD5SUM stuff to make room.
// Version 1.12: Added signatures for ATtiny2313A, ATtiny4313, ATtiny13
// Version 1.13: Added signature for Atmega8A
// Version 1.14: Added bootloader for Atmega8
// Version 1.15: Removed extraneous 0xFF from some files
// Version 1.16: Added signature for Atmega328
// Version 1.17: Allowed for running on the Leonardo, Micro, etc.
// Version 1.18: Added timed writing for Atmega8
// Version 1.19: Changed Atmega1280 to use the Optiboot loader.

/*

Copyright 2012 Nick Gammon.


PERMISSION TO DISTRIBUTE

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.


LIMITATION OF LIABILITY

The software is provided "as is", without warranty of any kind, express or implied,
including but not limited to the warranties of merchantability, fitness for a particular
purpose and noninfringement. In no event shall the authors or copyright holders be liable
for any claim, damages or other liability, whether in an action of contract,
tort or otherwise, arising from, out of or in connection with the software
or the use or other dealings in the software.

*/

#include <SPI.h>
#include <avr/pgmspace.h>
#include "programmer.h"

// const byte VUSB_SWITCH = 5;
// const byte MEGA_256RFR2_RESET = 6;
// const byte MEGA_16U2_RESET = 7;
// const byte DRIVER_FLASH_CS = SS;

// see Atmega datasheet for values
signatureType signatures[] =
{
//     signature          description   flash size  bootloader size
    { { 0x1E, 0xA8, 0x02 }, "ATmega256RFR2",  256 * kb,   8 * kb,
    (byte*)atmega256rfr2_bootloader,// loader image
    0x3E000,      // start address
    sizeof atmega256rfr2_bootloader,
    256,          // page size (for committing)
    0xFF,         // fuse low byte: external clock, max start-up time
    0xD0,         // fuse high byte: SPI enable, boot into bootloader, 8192 byte bootloader
    0xFE,         // fuse extended byte: brown-out detection at 2.7V
    0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.


// Atmega32U2 family
    { { 0x1E, 0x93, 0x89 }, "ATmega8U2",    8 * kb,   512 },
    { { 0x1E, 0x94, 0x89 }, "ATmega16U2",  16 * kb,   1 * kb,
    (byte*)atmega16u2_bootloader,// loader image
    0x0000,      // start address
    sizeof atmega16u2_bootloader,
    64,          // page size (for committing)
    0xEF,         // fuse low byte:
    0xD9,         // fuse high byte:
    0xF4,         // fuse extended byte:
    0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

    { { 0x1E, 0x95, 0x8A }, "ATmega32U2",  32 * kb,   512 },

// Atmega32U4 family
    { { 0x1E, 0x94, 0x88 }, "ATmega16U4",  16 * kb,   512 },
    { { 0x1E, 0x95, 0x87 }, "ATmega32U4",  32 * kb,   512 },


// ATtiny13 family
    { { 0x1E, 0x90, 0x07 }, "ATtiny13A",    1 * kb,   0,
    (byte*)attiny13a,// loader image
    0x0000,      // start address
    sizeof attiny13a,
    32,          // page size (for committing)
    0x2A,         // fuse low byte:
    0xFB,         // fuse high byte:
    0xFF,         // fuse extended byte:
    0x2F },       // lock bits: SPM is not allowed to write to the Boot Loader section.

};

// if signature found in above table, this is its index
AVRProgrammer::AVRProgrammer(int reset, int clockDivider) {
  foundSig = -1;
  resetPin = reset;
  lastAddressMSB = 0;
 
  digitalWrite(resetPin, HIGH);
  SPI.begin();
  SPI.setClockDivider(clockDivider);
  //pinMode(SCK, OUTPUT);

  // set up Timer 1
//  TCCR1A = _BV(COM1A0);  // toggle OC1A on Compare Match
//  TCCR1B = _BV(WGM12) | _BV(CS10);   // CTC, no prescaling
//  OCR1A =  0;       // output every cycle
}

void AVRProgrammer::startProgramming() {
  byte confirm;
  pinMode(resetPin, OUTPUT);
  //pinMode(SCK, OUTPUT);

  // we are in sync if we get back programAcknowledge on the third byte
  do {
    delay(100);
    // ensure SCK low
    digitalWrite(SCK, LOW);
    // then pulse reset, see page 309 of datasheet
    digitalWrite(resetPin, HIGH);
    delay(1);  // pulse for at least 2 clock cycles
    digitalWrite(resetPin, LOW);
    delay(25);  // wait at least 20 mS
    SPI.transfer(programEnable);
    SPI.transfer(programAcknowledge);
    confirm = SPI.transfer(0);
    SPI.transfer(0);
  } while (confirm != programAcknowledge);
  Serial.println("Entered programming mode OK.");
}

void AVRProgrammer::getSignature() {
  foundSig = -1;
  lastAddressMSB = 0;

  byte sig[3];
  Serial.print("Signature = ");
  for (byte i = 0; i < 3; i++) {
    sig[i] = program(readSignatureByte, 0, i);
    showHex(sig[i]);
  }  // end for each signature byte
  Serial.println();

  for (int j = 0; j < NUMITEMS(signatures); j++) {
    if (memcmp(sig, signatures[j].sig, sizeof sig) == 0) {
      foundSig = j;
      Serial.print("Processor = ");
      Serial.println(signatures[j].desc);
      Serial.print("Flash memory size = ");
      Serial.print(signatures[j].flashSize, DEC);
      Serial.println(" bytes.");
      if (signatures[foundSig].timedWrites) {
        Serial.print("Writes are timed, not polled.");
      }
      if (strncmp(signatures[j].desc, "ATtiny", 6) == 0) {
        SPI.setClockDivider(SPI_CLOCK_DIV64); // slow down SPI for the tinys
      }
      return;
    }
  }

  Serial.print("Unrecogized signature.");
}

void AVRProgrammer::getFuseBytes() {
  Serial.print("LFuse = ");
  showHex(program(readLowFuseByte, readLowFuseByteArg2), true);
  Serial.print("HFuse = ");
  showHex(program(readHighFuseByte, readHighFuseByteArg2), true);
  Serial.print("EFuse = ");
  showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2), true);
  Serial.print("Lock byte = ");
  showHex(program(readLockByte, readLockByteArg2), true);
  Serial.print("Clock calibration = ");
  showHex(program(readCalibrationByte), true);
}

void AVRProgrammer::writeFuseBytes(const byte lowFuse, const byte highFuse, const byte extendedFuse, const byte lockFuse) {
  Serial.println("Writing fuses ...");

  writeFuse(lowFuse, writeLowFuseByte);
  if (program(readLowFuseByte, readLowFuseByteArg2) != lowFuse) {
   Serial.print("Low fuse failed to write. Expected ");
   showHex(lowFuse, false, true);
   Serial.print("Got ");
   showHex(program(readLowFuseByte, readLowFuseByteArg2));
   Serial.println();
  } else {
    Serial.println("Successfully wrote low fuse");
  }
  
  writeFuse(highFuse, writeHighFuseByte);
  if (program(readHighFuseByte, readHighFuseByteArg2) != highFuse) {
   Serial.print("High fuse failed to write. Expected ");
   showHex(highFuse, false, true);
   Serial.print("Got ");
   showHex(program(readHighFuseByte, readHighFuseByteArg2));
   Serial.println();
  } else {
    Serial.println("Successfully wrote high fuse");
  }
  
  writeFuse(extendedFuse, writeExtendedFuseByte);
  if (program(readExtendedFuseByte, readExtendedFuseByteArg2) != extendedFuse) {
   Serial.print("Extended fuse failed to write. Expected ");
   showHex(extendedFuse, false, true);
   Serial.print("Got ");
   showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2));
   Serial.println();
  } else {
    Serial.println("Successfully wrote extended fuse");
  }
  
  writeFuse(lockFuse, writeLockByte);
  if (program(readLockByte, readLockByteArg2) != lockFuse) {
   Serial.print("Lock fuse failed to write. Expected ");
   showHex(lockFuse, false, true);
   Serial.print("Got ");
   showHex(program(lockFuse, readLockByteArg2));
   Serial.println();
  } else {
    Serial.println("Successfully wrote lock fuse");
  }

  // confirm them
  getFuseBytes();
}

// burn the bootloader to the target device
void AVRProgrammer::writeProgram(unsigned long loaderStart, const byte *image, const int length) {

  if (image == 0) {
    Serial.println("No bootloader support for this device.");
    return;
  }

  int i;
  byte lFuse = program(readLowFuseByte, readLowFuseByteArg2);

  byte newlFuse = signatures[foundSig].lowFuse;
  byte newhFuse = signatures[foundSig].highFuse;
  byte newextFuse = signatures[foundSig].extFuse;
  byte newlockByte = signatures[foundSig].lockByte;

  //unsigned long addr = signatures[foundSig].loaderStart;
  unsigned long addr = loaderStart;
  unsigned int  len = length;
  unsigned long pagesize = signatures[foundSig].pageSize;
  unsigned long pagemask = ~(pagesize - 1);
  const byte * flash = image;

  Serial.print("Bootloader page size = ");
  Serial.println(pagesize);
  Serial.print("Bootloader address = 0x");
  Serial.println(addr, HEX);
  Serial.print("Bootloader length = ");
  Serial.print(len);
  Serial.println(" bytes.");

  byte subcommand = 'U';

  unsigned long oldPage = addr & pagemask;

    Serial.println("Erasing chip...");
    program(programEnable, chipErase);   // erase it
    delay(20);  // for Atmega8
    pollUntilReady();
    Serial.println("Writing program...");
      
    for (i = 0; i < len; i += 2) {
      unsigned long thisPage = (addr + i) & pagemask;
      // page changed? commit old one
      if (thisPage != oldPage) {
        commitPage(oldPage);
        oldPage = thisPage;
      }
      writeFlash(addr + i, pgm_read_byte(flash + i));
      writeFlash(addr + i + 1, pgm_read_byte(flash + i + 1));
      /*
      Serial.print("Wrote to address ");
      showHex(addr + i);
      Serial.print(": ");
      showHex(pgm_read_byte(bootloader + i));
      Serial.println("");
      Serial.print("Wrote to address ");
      showHex(addr + i + 1);
      Serial.print(": ");
      showHex(pgm_read_byte(bootloader + i + 1));
      Serial.println("");
      */
    }  // end while doing each word

  // commit final page
  commitPage(oldPage);
  Serial.println("Written.");

  Serial.println("Verifying ...");

  // count errors
  unsigned int errors = 0;
  
  // check each byte
  for (i = 0; i < len; i++) {
    byte found = readFlash(addr + i);
    byte expected = pgm_read_byte(flash + i);
    if (found != expected) {
      if (errors <= 100) {
        Serial.print("Verification error at address ");
        showHex(addr + i, false, true);
        Serial.print(": Got: ");
        showHex(found);
        Serial.print(" Expected: ");
        showHex(expected, true);
      }
      errors++;
    }
  }

  if (errors == 0) {
    Serial.println("No errors found.");
  } else {
    Serial.print(errors, DEC);
    Serial.println(" verification error(s).");
    if (errors > 100) {
      Serial.print("First 100 shown.");
    }
    //return;  // don't change fuses if errors
  }
  Serial.print("Done.");
}

void AVRProgrammer::readProgram() {
  unsigned long addr = 0;
  unsigned int  len = 256;
  Serial.println();
  Serial.print("First 256 bytes of program memory:");
  Serial.println();

  for (int i = 0; i < len; i++) {
    // show address
    if (i % 16 == 0) {
      Serial.print(addr + i, HEX);
      Serial.print(": ");
    }
    showHex(readFlash(addr + i));
    // new line every 16 bytes
    if (i % 16 == 15) {
      Serial.println();
    }
  }
  Serial.println();
}

void AVRProgrammer::end() {
  digitalWrite(resetPin, HIGH);
}

bool AVRProgrammer::foundSignature() {
  return foundSig;
}

// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
byte AVRProgrammer::program(const byte b1, const byte b2, const byte b3, const byte b4) {
  SPI.transfer(b1);
  SPI.transfer(b2);
  SPI.transfer(b3);
  return SPI.transfer(b4);
}

// read a byte from flash memory
byte AVRProgrammer::readFlash(unsigned long addr) {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    program(loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }

  return program(readProgramMemory | high, highByte (addr), lowByte (addr));
}

// write a byte to the flash memory buffer (ready for committing)
byte AVRProgrammer::writeFlash(unsigned long addr, const byte data) {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  program (loadProgramMemory | high, 0, lowByte (addr), data);
}

// convert a boolean to Yes/No
void AVRProgrammer::showYesNo(const boolean b, const boolean newline) {
  if (b) {
    Serial.print("Yes");
  }
  else {
    Serial.print("No");
  }
  if (newline) {
    Serial.println();
  }
}

// poll the target device until it is ready to be programmed
void AVRProgrammer::pollUntilReady() {
  if (signatures[foundSig].timedWrites) {
    delay (10);  // at least 2 x WD_FLASH which is 4.5 mS
  } else {
    while ((program(pollReady) & 1) == 1) {}  // wait till ready
  }
}

// commit page
void AVRProgrammer::commitPage(unsigned long addr) {
  //Serial.print("Committing page starting at 0x");
  //Serial.print(addr, HEX);

  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }

  program(writeProgramMemory, highByte(addr), lowByte(addr));
  pollUntilReady();
}

// write specified value to specified fuse/lock byte
void AVRProgrammer::writeFuse(const byte newValue, const byte instruction) {
  if (newValue == 0) {
    return;  // ignore
  }

  program(programEnable, instruction, 0, newValue);
  pollUntilReady();
}

// show a byte in hex with leading zero and optional newline
void AVRProgrammer::showHex(const byte b, const boolean newline, const boolean show0x) {
  if (show0x) {
    Serial.print("0x");
  }
  // try to avoid using sprintf
  char buf[4] = { ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', ' ' , 0 };
  if (buf[0] > '9') {
    buf[0] += 7;
  }
  if (buf[1] > '9') {
    buf[1] += 7;
  }
  Serial.print(buf);
  if (newline) {
    Serial.println();
  }
}

