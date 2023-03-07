int pin = 3;
unsigned long duration;

void setup() {
  Serial.begin(9600);
  pinMode(pin, OUTPUT);
}

void loop() {
  digitalWrite(3,HIGH);
  delay(500);
  digitalWrite(3,LOW);
  delay(0);
}
