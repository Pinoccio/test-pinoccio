/*
Set fuses to:
Low: FF
High: D1
Extended: FC
*/

#include <Wire.h>

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
//  PORTB = 0x00;
//  DDRB = 0x00;
//  PORTC = 0x00;
//  DDRC = 0x00;
//  PORTD = 0x00;
//  DDRD = 0x00;
  Serial.begin(115200);
  Serial.println("Starting up...");
    
  for (int i=2; i<12; i++) {
    digitalWrite(i, LOW);
    pinMode(i, INPUT);
  }
  
  
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
  Serial.print("Received command: ");
  
  while (Wire.available()) { 
    buffer[ctr++] = Wire.read();
  }
  Serial.print(buffer);
  
  if (strncmp((const char*)buffer, "?", 1) == 0) {
    Serial.println(" (handshake)");
    command = HANDSHAKE;
  } else if (strncmp((const char*)buffer, "RA", 2) == 0) {
    Serial.println(" (analogPinRead)");
    command = ANALOG_READ;
  } else if (strncmp((const char*)buffer, "RD", 2) == 0) {
    Serial.println(" (digitalPinRead)");
    command = DIGITAL_READ;
  } else if (strncmp((const char*)buffer, "WD", 2) == 0) {
    Serial.println(" (digitalPinWrite)");
    command = DIGITAL_WRITE;
  } else {
    Serial.println(" (Unknown command)");
    command = UNKNOWN;
    //Wire.write("bad command"); 
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  char buf[2];
  buf[1] = 0;
  switch (command) {
    case HANDSHAKE:      
      handshake();
      break;
    case ANALOG_READ:
      buf[0] = buffer[ctr-1];
      analogPinRead(atoi(buf));
      break;
    case DIGITAL_READ:
      buf[0] = buffer[ctr-1];
      digitalPinRead(atoi(buf));
      break;
    case DIGITAL_WRITE:
      char buf2[2];
      buf2[0] = buffer[ctr-2];
      buf2[1] = 0;
      buf[0] = buffer[ctr-1];
      digitalPinWrite(atoi(buf2), atoi(buf));
      break;
    case UNKNOWN:
      Wire.write("Unknown");
  }
}

void handshake() {
  Serial.println("Sending back 'XYZ'");
  Wire.write("XYZ");
}

void analogPinRead(int pin) {
  Serial.print("Read A");
  Serial.println(pin);
  
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  
  int val = analogRead(pin);
  byte buf[3];
  buf[0] = ':';
  buf[1] = val >> 8;
  buf[2] = val & 0xFF;
  
  Serial.print("Sending back: ");
  Serial.write(buf, 2);
  Serial.println();
  Wire.write(buf, 2);
}

void digitalPinRead(int pin) {
  Serial.print("Read D");
  Serial.println(pin);
  Serial.println(digitalRead(pin));
  
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  
  itoa(digitalRead(pin), out, 10);
  output = strcat(":", out);
  
  Serial.print("Sending back: ");
  Serial.println(output);
  Wire.write(output);
}

void digitalPinWrite(int pin, int value) {
  Serial.print("Writing D");
  Serial.print(pin);
  Serial.print(" to ");
  Serial.println(value);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value);
  
  Serial.println("Sending back: 1");
  Wire.write(":1");
}

