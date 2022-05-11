int motor1pin1 = 26; //26
int motor1pin2 = 27; //27

int motor2pin1 = 18;
int motor2pin2 = 19;

// int enable_pin_a = 4;
// int enable_pin_b = 5;

void setup() {
  // put your setup code here, to run once:
pinMode(motor1pin1, OUTPUT);
pinMode(motor1pin2, OUTPUT);

pinMode(motor2pin1, OUTPUT);
pinMode(motor2pin2, OUTPUT);

pinMode(4, OUTPUT);
pinMode(5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

analogWrite(4, 200);
analogWrite(5, 200);

digitalWrite(motor1pin1, HIGH);
digitalWrite(motor1pin2, LOW);

digitalWrite(motor2pin1, HIGH);
digitalWrite(motor2pin2, LOW);
delay(1000);

digitalWrite(motor1pin1, LOW);
digitalWrite(motor1pin2, HIGH);

digitalWrite(motor2pin1, LOW);
digitalWrite(motor2pin2, HIGH);
delay(1000);

}
