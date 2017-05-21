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
  if(volts > 0) {
    analogWrite(MOTOR_LS_FWD, dc);
    analogWrite(MOTOR_LS_REV, 0);
  } else {
    analogWrite(MOTOR_LS_REV, dc);
    analogWrite(MOTOR_LS_FWD, 0);
  }
}


/* ----- plucking assembly motor stuff ----- */

#define MOTOR_PA_FWD 9
#define MOTOR_PA_REV 10

bool pa_onleft = true;

void pluck()
{
  if(pa_onleft) {
    digitalWrite(MOTOR_PA_FWD, HIGH);
  } else {
    digitalWrite(MOTOR_PA_REV, HIGH);
  }
  MsTimer2::start();
}

void __epluck()
{
  pa_onleft = !pa_onleft;
  digitalWrite(MOTOR_PA_FWD, LOW);
  digitalWrite(MOTOR_PA_REV, LOW);
  MsTimer2::stop();
}

/* ----- i2c position feedback ----- */

#define I2C_SLAVE_ADDR  0x4
#define SLAVE_RESET_PIN 2
#define CMD_CLRPOS 0x2

int16_t get_count()
{
  Wire.requestFrom(I2C_SLAVE_ADDR, 2);
  uint8_t low_byte = Wire.read();
  uint8_t high_byte = Wire.read();
  return (int16_t)(high_byte << 8) | low_byte;
}

int16_t zero_count()
{
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(CMD_CLRPOS);
  Wire.endTransmission();
}

/* ------ pid stuff -------- */

double input;
double e = 0;
double setpoint = 0;
double integral = 0;
double prop = 0;
double I = 7;
double P = 140;

#define CURPOS (((double)get_count()/100) * 0.008)
#define dt (0.0026)

double pid( double y, double r, double oldError )
{
  double error = r - y;

  integral += I*(dt*((error-oldError)) + dt*oldError);
  prop = P*error;

  input = integral + prop;

  return error;
}


void home()
{
  analogWrite(MOTOR_LS_REV, 0x80);
  int16_t old_pos = get_count();
  delay(4000);
  int16_t new_pos = get_count();

  for(int i = 0; i < 2; i ++) {
    do {
      delay(4000);
      old_pos = new_pos;
      new_pos = get_count();
    } while(abs(old_pos - new_pos) > 10) ;
  }
  
  analogWrite(MOTOR_LS_REV, 0x00);

  // home the pa motor
  analogWrite(MOTOR_PA_FWD, 0xe0);
  delay(2400);
  analogWrite(MOTOR_PA_FWD, 0x00);
  zero_count();
}


/* ----- MIDI stuff ----- */

MIDI_CREATE_DEFAULT_INSTANCE();

void midi_noteon(uint8_t chan, uint8_t pitch, uint8_t vel)
{
  setpoint = (double)pitch/127 * 0.3;
  pluck();
}

void setup()
{
  Serial.begin(115200);

  /* Setup MIDI */
  MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  MIDI.setHandleNoteOn(midi_noteon);
 

  /* output PWM (lead screw motor) */
  pinMode(MOTOR_LS_FWD, OUTPUT);
  pinMode(MOTOR_LS_REV, OUTPUT);
  
    /* I2C */
  Wire.begin();
  pinMode(SLAVE_RESET_PIN, OUTPUT);


  TCCR0B = (TCCR0B & 0b11111000) | 0x02; //  set PWM to 31k
  
  home();

  /* output PWM (plucking assy motor) */
  pinMode(MOTOR_PA_FWD, OUTPUT);
  pinMode(MOTOR_PA_REV, OUTPUT);
  MsTimer2::set(200, __epluck);



}

void loop()
{
  double x = CURPOS;
  e = pid(x, setpoint, e);
  set_drive(input);
  MIDI.read();
}
