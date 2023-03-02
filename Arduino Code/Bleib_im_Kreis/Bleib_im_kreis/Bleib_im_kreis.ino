void setup()
{
  pinMode(3, OUTPUT);
}

void loop()
{
  digitalWrite(3, HIGH);
  delayMicroseconds(400);l // Approximately 10% duty cycle @ 1KHz
  digitalWrite(3, LOW);
  delayMicroseconds(400);
}