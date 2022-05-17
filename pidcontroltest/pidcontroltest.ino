#define echoPin 4 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 5 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int speed; // variable for the distance measurement

//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input;
double output;
double SetPoint;
double cumError;
 double rateError;
void setup(){
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
      pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
      SetPoint = 0;                          //set point at zero degrees
       Serial.begin(9600);   
}    
 
void loop(){
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
  
  Serial.print(distance);
  Serial.println(" cm");
        input = analogRead(A0);                //read from rotary encoder connected to A0
        output = computePID(input);
        delay(500);

        if(distance>100){
        speed = 0;
        }
        if(distance<14){
        speed = 40;
        }
        if(distance<12){
        speed = 70;
        }
        if(distance<10){
        speed = 80;
        }
        if(distance<6){
        speed = 190;
        }
        if(distance<3){
        speed = 250;
        }
        if(distance>500){
        speed = 250;
        }


        Serial.println("SPEED");
        Serial.println(speed);
        analogWrite(3, speed);                //control the motor based on PID value

 
}
 
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = SetPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
