#include <Wire.h>

#define enc_A 2
#define enc_B 3
#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit address (remember to change this when adapting this example)

volatile int16_t count = 0;

void requestEvent()
{  
  // split 16-bit int into 2 8-bit int and send over I2C 
  uint8_t high_byte = count >> 8;
  uint8_t low_byte = count & 0xff;
  Wire.write(low_byte);
  Wire.write(high_byte);
}

void setup() {
  // Encoder input interrupt setup
  pinMode(enc_A, INPUT_PULLUP);
  pinMode(enc_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_A), A_count, RISING);

  //Serial.begin(9600);

  // I2C setup
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
}

void loop() {
}

// Interrupt routine
void A_count()
{
  count += (digitalRead(enc_A) == digitalRead(enc_B) ? 1 : -1);
}
