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

//#define AVR_PROGRAMMER_DEBUG
#ifdef AVR_PROGRAMMER_DEBUG
#  define PD(x) x
#else
#  define PD(x)
#endif

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

    { { 0x1E, 0x95, 0x8A }, "ATmega32U2",  32 * kb,   1 * kb,
    (byte*)atmega16u2_bootloader,// loader image
    0x0000,      // start address
    sizeof atmega16u2_bootloader,
    64,          // page size (for committing)
    0xEF,         // fuse low byte:
    0xD9,         // fuse high byte:
    0xF4,         // fuse extended byte:
    0x2F },       // lock bits: SPM is not allowed to wri

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
}

void AVRProgrammer::startProgramming() {
  byte confirm;
  pinMode(resetPin, OUTPUT);

  // we are in sync if we get back programAcknowledge on the third byte
  do {
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
  PD(Serial1.println("Entered programming mode OK"));

  // we must set extended address byte to 0 to start, otherwise our writes start at 128k, not 0
  program(loadExtendedAddressByte, 0, lastAddressMSB);
}

void AVRProgrammer::getSignature() {
  foundSig = -1;
  lastAddressMSB = 0;

  byte sig[3];
  PD(Serial1.print("Signature = "));
  for (byte i = 0; i < 3; i++) {
    sig[i] = program(readSignatureByte, 0, i);
    PD(showHex(sig[i]));
  }
  PD(Serial1.println());

  for (int j = 0; j < NUMITEMS(signatures); j++) {
    if (memcmp(sig, signatures[j].sig, sizeof sig) == 0) {
      foundSig = j;
      PD(Serial1.print("Processor = "));
      PD(Serial1.println(signatures[j].desc));
      PD(Serial1.print("Flash memory size = "));
      PD(Serial1.print(signatures[j].flashSize, DEC));
      PD(Serial1.println(" bytes."));
      if (signatures[foundSig].timedWrites) {
        PD(Serial1.print("Writes are timed, not polled."));
      }
      if (strncmp(signatures[j].desc, "ATtiny", 6) == 0) {
        SPI.setClockDivider(SPI_CLOCK_DIV64); // slow down SPI for the tinys
      }
      return;
    }
  }

  PD(Serial1.print("Unrecogized signature."));
}

void AVRProgrammer::getFuseBytes() {
  PD(Serial1.print("LFuse = "));
  PD(showHex(program(readLowFuseByte, readLowFuseByteArg2), true));
  PD(Serial1.print("HFuse = "));
  PD(showHex(program(readHighFuseByte, readHighFuseByteArg2), true));
  PD(Serial1.print("EFuse = "));
  PD(showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2), true));
  PD(Serial1.print("Lock byte = "));
  PD(showHex(program(readLockByte, readLockByteArg2), true));
  PD(Serial1.print("Clock calibration = "));
  PD(showHex(program(readCalibrationByte), true));
}

void AVRProgrammer::writeFuseBytes(const byte lowFuse, const byte highFuse, const byte extendedFuse, const byte lockFuse) {
  PD(Serial1.println("Writing fuses ..."));

  writeFuse(lowFuse, writeLowFuseByte);
  if (program(readLowFuseByte, readLowFuseByteArg2) != lowFuse) {
    PD(Serial1.print("Low fuse failed to write. Expected "));
    PD(showHex(lowFuse, false, true));
    PD(Serial1.print("Got "));
    PD(showHex(program(readLowFuseByte, readLowFuseByteArg2)));
    PD(Serial1.println());
  } else {
    PD(Serial1.println("Successfully wrote low fuse"));
  }

  writeFuse(highFuse, writeHighFuseByte);
  if (program(readHighFuseByte, readHighFuseByteArg2) != highFuse) {
    PD(Serial1.print("High fuse failed to write. Expected "));
    PD(showHex(highFuse, false, true));
    PD(Serial1.print("Got "));
    PD(showHex(program(readHighFuseByte, readHighFuseByteArg2)));
    PD(Serial1.println());
  } else {
    PD(Serial1.println("Successfully wrote high fuse"));
  }

  writeFuse(extendedFuse, writeExtendedFuseByte);
  if (program(readExtendedFuseByte, readExtendedFuseByteArg2) != extendedFuse) {
    PD(Serial1.print("Extended fuse failed to write. Expected "));
    PD(showHex(extendedFuse, false, true));
    PD(Serial1.print("Got "));
    PD(showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2)));
    PD(Serial1.println());
  } else {
    PD(Serial1.println("Successfully wrote extended fuse"));
  }

  writeFuse(lockFuse, writeLockByte);
  if (program(readLockByte, readLockByteArg2) != lockFuse) {
    PD(Serial1.print("Lock fuse failed to write. Expected "));
    PD(showHex(lockFuse, false, true));
    PD(Serial1.print("Got "));
    PD(showHex(program(lockFuse, readLockByteArg2)));
    PD(Serial1.println());
  } else {
    PD(Serial1.println("Successfully wrote lock fuse"));
  }

  // confirm them
  getFuseBytes();
}

// burn the bootloader to the target device
void AVRProgrammer::writeProgram(unsigned long loaderStart, const byte *image, const int length) {

  if (image == 0) {
    PD(Serial1.println("No bootloader support for this device."));
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

  PD(Serial1.print("Bootloader page size = "));
  PD(Serial1.println(pagesize));
  PD(Serial1.print("Bootloader address = 0x"));
  PD(Serial1.println(addr, HEX));
  PD(Serial1.print("Bootloader length = "));
  PD(Serial1.print(len));
  PD(Serial1.println(" bytes."));

  byte subcommand = 'U';

  unsigned long oldPage = addr & pagemask;

  PD(Serial1.println("Writing program..."));

  for (i = 0; i < len; i += 2) {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage) {
      commitPage(oldPage);
      oldPage = thisPage;
    }
    writeFlash(addr + i, pgm_read_byte(flash + i));
    writeFlash(addr + i + 1, pgm_read_byte(flash + i + 1));
  }

  // commit final page
  commitPage(oldPage);
  PD(Serial1.println("Written."));
  PD(Serial1.println("Verifying ..."));

  // count errors
  unsigned int errors = 0;

  // check each byte
  for (i = 0; i < len; i++) {
    byte found = readFlash(addr + i);
    byte expected = pgm_read_byte(flash + i);
    if (found != expected) {
      if (errors <= 100) {
        PD(Serial1.print("Verification error at address ");
        PD(showHex(addr + i, false, true)));
        PD(Serial1.print(": Got: "));
        PD(showHex(found));
        PD(Serial1.print(" Expected: "));
        PD(showHex(expected, true));
      }
      errors++;
    }
  }

  if (errors == 0) {
    PD(Serial1.println("No errors found."));
  } else {
    PD(Serial1.print(errors, DEC));
    PD(Serial1.println(" verification error(s)."));
    if (errors > 100) {
      PD(Serial1.print("First 100 shown."));
    }
    //return;  // don't change fuses if errors
  }
  PD(Serial1.print("Done."));
}

void AVRProgrammer::writeProgramFromSerialFlash(uint32_t loaderStart, FlashClass *flash, const uint32_t flashAddress, const uint32_t length) {

  uint32_t i;
  uint32_t addr = loaderStart;
  uint32_t  len = length;
  uint32_t pagesize = signatures[foundSig].pageSize;
  uint32_t pagemask = ~(pagesize - 1);

  uint32_t timesThrough = 0;
  uint16_t bufCtr = 0;
  uint16_t verifyCtr = 0;
  uint16_t bufLen = 16384;
  byte flashBuffer[bufLen];

  // count errors
  unsigned int errors = 0;

  PD(Serial1.print("Bootloader page size = "));
  PD(Serial1.println(pagesize));
  PD(Serial1.print("Bootloader address = 0x"));
  PD(Serial1.println(addr, HEX));
  PD(Serial1.print("Bootloader length = "));
  PD(Serial1.print(len));
  PD(Serial1.println(" bytes"));

  byte subcommand = 'U';

  unsigned long oldPage = addr & pagemask;

  PD(Serial1.println("Writing program..."));

  // load each word
  for (i=0; i<len; i+=2, bufCtr+=2) {

    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage) {
      commitPage(oldPage);
      oldPage = thisPage;
    }

    if (i % bufLen == 0) {
      // read new chunk of bytes into buffer
      bufCtr = 0;
      flash->read(flashAddress + i, &flashBuffer, bufLen);
      startProgramming();

      timesThrough++;
      PD(Serial1.print("Writing "));
      if ((timesThrough * bufLen) > len) {
        PD(Serial1.print(len));
      } else {
        PD(Serial1.print(timesThrough * bufLen));
      }
      PD(Serial1.print(" bytes of a total of "));
      PD(Serial1.println(len));
    }

    writeFlash(addr + i, flashBuffer[bufCtr]);
    writeFlash(addr + i + 1, flashBuffer[bufCtr + 1]);
  }

  // commit final page
  commitPage(oldPage);

  PD(Serial1.println("Written."));

  // check each byte
  for (i=0; i<len; i++, bufCtr++) {
    if (i % bufLen == 0) {
      PD(Serial1.print("Verifying flash at address: 0x"));
      PD(Serial1.println(loaderStart + i, HEX));
      bufCtr = 0;
      flash->read(flashAddress + i, &flashBuffer, bufLen);
      startProgramming();
    }

    byte found = readFlash(addr + i);
    if (found != flashBuffer[bufCtr]) {
      if (errors <= 10) {
        PD(Serial1.print("Verification error at address 0x"));
        PD(Serial1.print(addr + i, HEX));
        PD(Serial1.print(": Got: "));
        PD(showHex(found, false, true));
        PD(Serial1.print("Expected: "));
        PD(showHex(flashBuffer[bufCtr], false, true));
        PD(Serial1.println(bufCtr));
      }
      errors++;
    }
  }

  if (errors == 0) {
    PD(Serial1.println("No errors found."));
  } else {
    PD(Serial1.print(errors, DEC));
    PD(Serial1.println(" verification error(s)."));
    if (errors > 100) {
      PD(Serial1.print("First 100 shown."));
    }
  }
  PD(Serial1.print("Done."));
}

void AVRProgrammer::readProgram(uint32_t address, uint32_t length) {
  PD(Serial1.println());
  PD(Serial1.print("Reading "));
  PD(Serial1.print(length));
  PD(Serial1.print(" bytes of program memory starting at address 0x"));
  PD(Serial1.println(address, HEX));

  for (uint32_t i = 0; i < length; i++) {
    // show address
    if (i % 16 == 0) {
      PD(Serial1.print(address + i, HEX));
      PD(Serial1.print(": "));
    }
    PD(showHex(readFlash(address + i)));
    // new line every 16 bytes
    if (i % 16 == 15) {
      PD(Serial1.println());
    }
  }
  PD(Serial1.println());
}

uint8_t AVRProgrammer::readEeprom(uint32_t address) {
  PD(Serial1.println("Reading EEPROM..."));
  return program(readEepromMemory, (byte)address >> 16, (byte)address & 0xFFFF, 0);
}

void AVRProgrammer::writeEeprom(uint32_t address, uint8_t value) {
  PD(Serial1.println("Writing EEPROM..."));
  program(writeEepromMemory, address >> 16, address & 0xFFFF, value);
  pollUntilReady();
}

void AVRProgrammer::eraseChip() {
  PD(Serial1.println("Erasing chip..."));
  program(programEnable, chipErase);
  //delay(20);  // for Atmega8
  pollUntilReady();
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
//  Serial1.println("writeFlash: ");
//  Serial1.print("high byte: ");
//  Serial1.println(high);
//  Serial1.print("low byte: ");
//  Serial1.println(lowByte(addr));
//  Serial1.print("sending the following command: 0x");
//  Serial1.print(loadProgramMemory | high, HEX);
//  Serial1.print(", 0, 0x");
//  Serial1.print(lowByte(addr), HEX);
//  Serial1.print(", 0x");
//  Serial1.println(data, HEX);
  program(loadProgramMemory | high, 0, lowByte (addr), data);
}

// convert a boolean to Yes/No
void AVRProgrammer::showYesNo(const boolean b, const boolean newline) {
  if (b) {
    PD(Serial1.print("Yes"));
  } else {
    PD(Serial1.print("No"));
  }
  if (newline) {
    PD(Serial1.println());
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
  //Serial1.print("Committing page starting at 0x");
  //Serial1.print(addr, HEX);

  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }

//  Serial1.println("commitPage: ");
//  Serial1.print("sending the following command: 0x");
//  Serial1.print(writeProgramMemory, HEX);
//  Serial1.print(", 0, 0x");
//  Serial1.print(highByte(addr), HEX);
//  Serial1.print(", 0x");
//  Serial1.println(lowByte(addr), HEX);

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
    PD(Serial1.print("0x"));
  }
  // try to avoid using sprintf
  char buf[4] = { ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', ' ' , 0 };
  if (buf[0] > '9') {
    buf[0] += 7;
  }
  if (buf[1] > '9') {
    buf[1] += 7;
  }
  PD(Serial1.print(buf));
  if (newline) {
    PD(Serial1.println());
  }
}

