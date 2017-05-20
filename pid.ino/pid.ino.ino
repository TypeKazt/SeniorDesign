#include <MIDI.h>
#include <Wire.h>
#include "MsTimer2.h"



/* ----- lead screw motor ------ */

#define MOTOR_LS_FWD 5
#define MOTOR_LS_REV 6

void set_drive(double volts)
{
  if(volts > 12)  volts = 12;
  if(volts < -12) volts = -12;
  uint8_t dc = (uint8_t)((abs(volts)*255)/12);
  if(volts > 0)
    analogWrite(MOTOR_LS_FWD, dc);
  else analogWrite(MOTOR_LS_REV, dc);
}


/* ----- plucking assembly motor stuff ----- */

#define MOTOR_PA_FWD 9
#define MOTOR_PA_REV 10

bool pa_onleft = true;

void pluck()
{
  if(pa_onleft) {
    analogWrite(MOTOR_PA_FWD, 0xe0);
  } else {
    analogWrite(MOTOR_PA_REV, 0xe0);
  }
  MsTimer2::start();
}

void __epluck()
{
  pa_onleft = !pa_onleft;
  analogWrite(MOTOR_PA_FWD, 0x00);
  analogWrite(MOTOR_PA_REV, 0x00);
  MsTimer2::stop();
}

MIDI_CREATE_DEFAULT_INSTANCE();

void midi_noteon(uint8_t chan, uint8_t pitch, uint8_t vel)
{
  // TODO: set the position
  Serial.println("midi cmd");
  pluck();
}

/* ----- i2c position feedback ----- */

#define I2C_SLAVE_ADDR  0x4
#define SLAVE_RESET_PIN 2
#define CMD_CLRPOS 0x2

int16_t get_count()
{
  Wire.requestFrom(I2C_SLAVE_ADDR, 2);
  //Wire.write(CMD_REQPOS);
  uint8_t low_byte = Wire.read();
  uint8_t high_byte = Wire.read();
  return (int16_t) (high_byte << 8) | low_byte;
}

int16_t zero_count()
{
  //Wire.requestFrom(I2C_SLAVE_ADDR, 0);
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(CMD_CLRPOS);
  Wire.endTransmission();
}

/* ------ pid stuff -------- */

double input;
double e = 0;
double set = 0.2;
double integral = 0;
double prop = 0;
double I = 7;
double P = 80;

#define CURPOS (((double)get_count()/600) * 0.008)
#define dt (0.0026)

double pid( double y, double r, double oldError)
{
  double error = r - y;

  integral += I*(dt*(10*(error-oldError))) + dt*oldError;
  prop = P*error;

  input = integral + prop;

  return error;
}


void home()
{

  analogWrite(MOTOR_LS_REV, 0x80);
  int16_t old_pos = get_count();
  delay(500);
  int16_t new_pos = get_count();

  for(int i = 0; i < 2; i ++) {
    do {
      delay(500);
      old_pos = new_pos;
      new_pos = get_count();
    } while(abs(old_pos - new_pos) > 10) ;
  }
  
  analogWrite(MOTOR_LS_REV, 0x00);

  // home the pa motor
  analogWrite(MOTOR_PA_FWD, 0x90);
  delay(300);
  analogWrite(MOTOR_PA_FWD, 0x00);
  zero_count();
}

void setup()
{
  /* Setup MIDI */
  MIDI.begin();
  //MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  MIDI.setHandleNoteOn(midi_noteon);
  Serial.begin(115200);

  //Serial.begin(9600);
  Serial.println("------------------------");
  
  /* output PWM (lead screw motor) */
  pinMode(MOTOR_LS_FWD, OUTPUT);
  pinMode(MOTOR_LS_REV, OUTPUT);
 
  /* output PWM (plucking assy motor) */
  pinMode(MOTOR_PA_FWD, OUTPUT);
  pinMode(MOTOR_PA_REV, OUTPUT);
  MsTimer2::set(200, __epluck);

  /* I2C */
  /*Wire.begin();
  pinMode(SLAVE_RESET_PIN, OUTPUT);*/
  
  //home();
  //double x = CURPOS;
}

void loop()
{
  // Read incoming messages

/*
  double x = CURPOS;

  e = pid( x, set, e );
  set_drive( input );
  
  Serial.print(x*1000);
  Serial.print("  ");
  Serial.print(e*1000);
  Serial.print("  ");
  Serial.println(input);
*/
  
  if(MIDI.read())
  {
    Serial.println("MIDI received");
      delay(1000);
      pluck();
  }

}


