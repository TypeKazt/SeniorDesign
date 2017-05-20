#include "MIDI.h"
#include "MsTimer2.h"

MIDI_CREATE_DEFAULT_INSTANCE();

void midi_noteon(uint8_t chan, uint8_t pitch, uint8_t vel)
{
  // TODO: set the position
  //Serial.println("midi cmd");
  pluck();
}

/* plucking assembly motor */
#define MOTOR_PA_FWD 9
#define MOTOR_PA_REV 10

void pa_home()
{
  analogWrite(MOTOR_PA_FWD, 0x30);
  delay(1000);
  analogWrite(MOTOR_PA_FWD, 0x00);
}

bool pa_onleft;

void pluck()
{
  if(pa_onleft) {
    analogWrite(MOTOR_PA_FWD, 0xff);
  } else {
    analogWrite(MOTOR_PA_REV, 0xff);
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

void home()
{
  pa_onleft = true;
  analogWrite(MOTOR_PA_REV, 0xe0);
  delay(500);
  analogWrite(MOTOR_PA_REV, 0x00);
}

void setup()
{
  Serial.begin(115200);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(midi_noteon);
  
  MsTimer2::set(200, __epluck); 
  
  /* output PWM (plucking assy motor) */
  pinMode(MOTOR_PA_FWD, OUTPUT);
  pinMode(MOTOR_PA_REV, OUTPUT);
  home();
}
  
void loop()
{
  MIDI.read();
}


