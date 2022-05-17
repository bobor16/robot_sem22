#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#include <WiFi.h>
#include <esp_now.h>
#include <analogWrite.h>

Adafruit_MPU6050 mpu;

uint8_t broadcastAddress[] = {0x10, 0x97, 0xBD, 0xD4, 0x74, 0x50};
esp_now_peer_info_t peerInfo;
String success;

int incoming_cm;
int rotor_pin = 4;


float outgoing_x;
float outgoing_y;

const int hall_pin = 34;
int hall_effect;

typedef struct struct_distance {
  int cm;
} struct_distance;

typedef struct struct_message {
    float x;
    float y;
} struct_message;

struct_distance incoming_distance;

struct_message message;

// ** PID ** 
// constants
double kp = 2;
double ki = 5;
double kd = 1;
int speed; // variable for the distance measurement
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input;
double output;
double SetPoint;
double cumError;
 double rateError;
// ** PID ** 

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
   // success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len) {
    memcpy(&incoming_distance, incomingData, sizeof(incoming_distance));

  Serial.print("Bytes received: ");
  Serial.println(len);

  incoming_cm = incoming_distance.cm;
}

void setup() {
  SetPoint = 0;                    //PID:set point at zero degrees
  Serial.begin(115200);

  pinMode(rotor_pin, OUTPUT);
  
  pinMode(hall_pin, INPUT_PULLUP);
  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");
  
  WiFi.mode(WIFI_STA);
  
  
  // Init ESP_NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } 
  
  /* Once ESP-NOW is successfully init, we will register for send cb to 
     get the status of transmittet packets.
  */
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
    }
        // Register for a callback function that will be called when data is received
      esp_now_register_recv_cb(OnDataRecv);

  }

void loop() {

  //hall_effect = digitalRead(hall_pin);
  //Serial.print(hall_effect);

  Serial.print("cm: ");
  Serial.println(incoming_cm);
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  message.x = a.acceleration.x;
  message.y = a.acceleration.y;

  // Serial.println(a.acceleration.x);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));

  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  // ** PID ** 
          input = analogRead(A0);                //read from rotary encoder connected to A0
        output = computePID(input);
        delay(500);

        if(incoming_cm>100){
        speed = 0;
        }
        if(incoming_cm<14){
        speed = 40;
        }
        if(incoming_cm<12){
        speed = 70;
        }
        if(incoming_cm<10){
        speed = 80;
        }
        if(incoming_cm<8){
        speed = 120;
        }
        if(incoming_cm<6){
        speed = 190;
        }
        if(incoming_cm<3){
        speed = 250;
        }
        if(incoming_cm>500){
        speed = 250;
        }


        Serial.println("SPEED");
        Serial.println(speed);
        analogWrite(0, speed);                //control the motor based on PID value
        // ** PID ** 
  delay(100);
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
