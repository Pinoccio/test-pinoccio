/*
Set fuses to:
Low: FF
High: D1
Extended: FC
*/

#include <Wire.h>
#include <stdio.h>

//#define AVR_DEBUG
#ifdef AVR_DEBUG
#  define D(x) x
#else
#  define D(x)
#endif

enum {
  INIT,
  HANDSHAKE,
  DIGITAL_READ,
  DIGITAL_WRITE,
  ANALOG_READ,
  UNKNOWN
};

const char SLAVE_ADDR = 2;
char command = INIT;

// analog
#define VUSB   PC0  // A0
#define VBAT   PC1  // A1
#define VCC    PC2  // A2

// digital
#define TX1_T  PD0  // D0
#define A5_T   PD1  // D1
#define A6_T   PD2  // D2
#define A7_T   PD3  // D3
#define D2_T   PD4  // D4
#define D3_T   PD5  // D5
#define D4_T   PD6  // D6
#define D5_T   PD7  // D7
#define D6_T   PB0  // D8
#define D7_T   PB1  // D9
#define D8_T   PB2  // D10
#define SSN_T  PB4  // D12
#define SDA_T  PC3  // A3 (D17)
//#define SCL_T  ADC6 // A6 
//#define RX1_T  ADC7 // A7

char buffer[16];
int ctr;
char* output = (char *)malloc(256);
char out[8];
 
void setup() {
  for (int i=0; i<20; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  D(Serial.begin(115200));
  D(Serial.println("Starting up..."));
  
  Wire.begin(SLAVE_ADDR);          
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() { }

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  int pin;
  int value;
  ctr = 0;
  D(Serial.print("Received command: "));
  
  while (Wire.available()) { 
    buffer[ctr++] = Wire.read();
  }
  D(Serial.print(buffer));
  
  if (strncmp((const char*)buffer, "?", 1) == 0) {
    D(Serial.println(" (handshake)"));
    command = HANDSHAKE;
  } else if (strncmp((const char*)buffer, "RA", 2) == 0) {
    D(Serial.println(" (analogPinRead)"));
    command = ANALOG_READ;
  } else if (strncmp((const char*)buffer, "RD", 2) == 0) {
    D(Serial.println(" (digitalPinRead)"));
    command = DIGITAL_READ;
  } else if (strncmp((const char*)buffer, "WD", 2) == 0) {
    D(Serial.println(" (digitalPinWrite)"));
    command = DIGITAL_WRITE;
  } else {
    D(Serial.println(" (Unknown command)"));
    command = UNKNOWN;
    //Wire.write("bad command"); 
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  char buf[3];
  buf[2] = 0;
  switch (command) {
    case HANDSHAKE:      
      handshake();
      break;
    case ANALOG_READ:
      buf[0] = buffer[ctr-1];
      analogPinRead(atoi(buf));
      break;
    case DIGITAL_READ:
      buf[0] = buffer[ctr-2];
      buf[1] = buffer[ctr-1];
      digitalPinRead(atoi(buf));
      break;
    case DIGITAL_WRITE:
      char buf2[3];
      buf2[0] = buffer[ctr-3];
      buf2[1] = buffer[ctr-2];
      buf2[2] = 0;
      buf[0] = buffer[ctr-1];
      digitalPinWrite(atoi(buf2), atoi(buf));
      break;
    case UNKNOWN:
      Wire.write("Unknown");
  }
}

void handshake() {
  D(Serial.println("Sending back 'XYZ'"));
  Wire.write("XYZ");
}

void analogPinRead(int pin) {
  D(Serial.print("Read A"));
  D(Serial.println(pin));
 
  uint16_t val = analogRead(pin);
  
  sprintf(output, ":%04u", val);
  
  D(Serial.print("Sending back: "));
  D(Serial.println(output));
  Wire.write(output);
}

void digitalPinRead(int pin) {
  D(Serial.print("Read D"));
  D(Serial.println(pin));
  D(Serial.println(digitalRead(pin)));
  
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  
  itoa(digitalRead(pin), out, 10);
  output = strcat(":", out);
  
  D(Serial.print("Sending back: "));
  D(Serial.println(output));
  Wire.write(output);
}

void digitalPinWrite(int pin, int value) {
  D(Serial.print("Writing D"));
  D(Serial.print(pin));
  D(Serial.print(" to "));
  D(Serial.println(value));
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value);
  
  D(Serial.println("Sending back: 1"));
  Wire.write(":1");
}

