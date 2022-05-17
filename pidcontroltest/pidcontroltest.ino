/*
sketch belongs to this video: https://youtu.be/crw0Hcc67RY
write by Moz for YouTube changel logMaker360
4-12-2017
*/

#include <PID_v1.h>

double Setpoint ; // will be the desired value
double Input; // photo sensor
double Output ; //LED
//PID parameters
double Kp=0, Ki=10, Kd=0; 
 
//create PID instance 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define echoPin 4 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 5 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
 
void setup()
{
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600);   
  //Hardcode the brigdness value
  Setpoint = 75;
  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID.SetTunings(Kp, Ki, Kd);
}
 
void loop()
{

  //Read the value from the light sensor. Analog input : 0 to 1024. We map is to a value from 0 to 255 as it's used for our PWM function.
  Input = map(analogRead(5), 0, 1024, 0, 255);  // photo senor is set on analog pin 5
    Serial.print(analogRead(5));
  //PID calculation
  myPID.Compute();
  //Write the output as calculated by the PID function
  analogWrite(3,Output); //LED is set to digital 3 this is a pwm pin. 
  //Send data by serial for plotting 
 // Serial.print(Input);
//  Serial.print(" ");
 // Serial.println(Output);
  Serial.print(" ");  
 // Serial.println(Setpoint);
  delay(100); 
    // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  distance = distance * 3;
  Serial.print(distance);
  Serial.println(" cm");
}
