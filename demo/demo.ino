
void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  Serial.begin(9600);
}

void loop() {
	/*moveForward();
	delay(10);
	moveBack();
	delay(10);
}

void moveForward()
{
	digitalWrite(6, LOW);
  digitalWrite(10, LOW);
	for(int i = 0; i< 100; i++)
	{
		analogWrite(5, i);
    analogWrite(9, i);
		delay(4);
	}
	for(int i = 0; i< 100; i++)
	{
		analogWrite(5, 100 - i);
    analogWrite(9, 100 - i);
		delay(4);
	}
}

void moveBack()
{
	digitalWrite(5, LOW);
  digitalWrite(9, LOW);
	for(int i = 0; i< 100; i++)
	{
		analogWrite(6, i);
    analogWrite(10, i);
		delay(4);
	}
	for(int i = 0; i< 100; i++)
	{
		analogWrite(6, 100 - i);
    analogWrite(10, 100 - i);
		delay(4);
	}
}
