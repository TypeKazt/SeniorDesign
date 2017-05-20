#include <Wire.h>

#define enc_A 2 //portd 2
#define enc_B 3 //portd 3

#define CMD_CLRPOS 0x2

#define I2C_SLAVE_ADDRESS 0x4

volatile int16_t count = 0;

void requestEvent()
{
  //Serial.println(count);
  Wire.write(count & 0xff);
  Wire.write(count >> 8);
}

void receiveEvent(int d)
{
  Serial.println("CLR ------------------------------------------------");
  uint8_t cmd = Wire.read();
  switch(cmd){
    case CMD_CLRPOS:
      count = 0;
      break;
  }
}

void setup() {
  //DDRD = 00000010;  // pins 2 to 7 as outputs

  // Encoder input interrupt setup
  pinMode(enc_A, INPUT_PULLUP);
  pinMode(enc_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_A), A_count, RISING);


  Serial.begin(9600);

  // I2C setup
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {
}

// Interrupt routine
void A_count()
{
  //bool a = bitRead(PIND, enc_A);
  //bool b = bitRead(PIND, enc_B);
  
  //Serial.print(a);
  //Serial.print(" ");
  //Serial.println(b);
  count += ( bitRead(PIND, enc_A) == bitRead(PIND, enc_B) ? 1 : -1);
  //count += (digitalRead( enc_A) == digitalRead(enc_B) ? 1 : -1);
}
