#include <MIDI.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR  0x4
#define SLAVE_RESET_PIN 2


 // Created and binds the MIDI interface to the default hardware Serial port
MIDI_CREATE_DEFAULT_INSTANCE();

int16_t get_count()
{
	Wire.requestFrom(I2C_SLAVE_ADDR, (int) 2);
	int16_t result = Wire.read();
	result += Wire.read() << 8;
	return result;
}

void setup()
{
	// Setup MIDI
	MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
	Serial.begin(115200);

	// INIT the wire object for I2C
	Wire.begin();
	pinMode(SLAVE_RESET_PIN, OUTPUT);

	// reset slave
	//digitalWrite(SLAVE_RESET_PIN, LOW);
	//delay(500);
	//digitalWrite(SLAVE_RESET_PIN, HIGH);
	
	// Wait for slave to reset
	//delay(10000);
}

void loop()
{
	// Read incoming messages
	/*
	if(MIDI.read())
	 Serial.println("MIDI received");
	*/

	delay(1000); // For testing 
	Serial.println(get_count());
}
