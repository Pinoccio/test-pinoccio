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
#include "utility/Flash.h"

// hex bootloader data
#include "bootloader_atmega16u2.h"
#include "bootloader_atmega256rfr2.h"
#include "bootloader_attiny13a.h"

// number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// programming commands to send via SPI to the chip
enum {
  programEnable = 0xAC,

  // writes are preceded by programEnable
  chipErase = 0x80,
  writeLockByte = 0xE0,
  writeLowFuseByte = 0xA0,
  writeHighFuseByte = 0xA8,
  writeExtendedFuseByte = 0xA4,

  pollReady = 0xF0,

  programAcknowledge = 0x53,

  readSignatureByte = 0x30,
  readCalibrationByte = 0x38,

  readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
  readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
  readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,
  readLockByte = 0x58,          readLockByteArg2 = 0x00,

  readProgramMemory = 0x20,
  writeProgramMemory = 0x4C,
  loadExtendedAddressByte = 0x4D,
  loadProgramMemory = 0x40,

};

// structure to hold signature and other relevant data about each chip
typedef struct {
  byte sig [3];
  char * desc;
  unsigned long flashSize;
  unsigned int baseBootSize;
  byte * bootloader;
  unsigned long loaderStart;  // bytes
  unsigned int loaderLength;  // bytes
  unsigned long pageSize;     // bytes
  byte lowFuse, highFuse, extFuse, lockByte;
  byte timedWrites;    // if pollUntilReady won't work by polling the chip
} 
signatureType;

const unsigned long kb = 1024;

class AVRProgrammer {
public:
  AVRProgrammer(int reset, int clockDivider=SPI_CLOCK_DIV4);
  void startProgramming();
  void getSignature();
  void getFuseBytes();
  void writeFuseBytes(const byte lowFuse, const byte highFuse, const byte extendedFuse, const byte lockFuse=0xFF);
  void writeProgram(unsigned long loaderStart, const byte *image, const int length);
  void writeProgramFromSerialFlash(uint32_t loaderStart, FlashClass *flash, const uint32_t flashAddress, const uint32_t length);
  void readProgram(uint32_t address, uint32_t length);
  void eraseChip();
  bool foundSignature();
  void end();

protected:
  byte program(const byte b1, const byte b2=0, const byte b3=0, const byte b4=0);
  byte readFlash(unsigned long addr);
  byte writeFlash(unsigned long addr, const byte data);
  void showYesNo(const boolean b, const boolean newline = false);
  void pollUntilReady();
  void commitPage(unsigned long addr);    
  void writeFuse(const byte newValue, const byte instruction);
  void showHex(const byte b, const boolean newline = false, const boolean show0x = true);

  int foundSig;
  int resetPin;
  byte lastAddressMSB;
  byte spiSpeed;
};

