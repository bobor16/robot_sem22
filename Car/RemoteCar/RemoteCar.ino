#include <WiFi.h>
#include <esp_now.h>

/* h-bridge */
// left motor a
int motor1pin1 = 26;
int motor1pin2 = 27;

// right motor b
int motor2pin1 = 18;
int motor2pin2 = 19;

// enable a and b motor
int enable_pin_a = 4;
int enable_pin_b = 5;

// instansiate mac address of the controller
uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0xD6, 0xC5, 0xE4};

float incoming_x;
float incoming_y;

typedef struct struct_message {
    float x;
    float y;
} struct_message;

struct_message message;

struct_message incomingMessage;

esp_now_peer_info_t peerInfo;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&message, incomingData, sizeof(message));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
 
  incoming_x = message.x;
  incoming_y = message.y;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // set h-bridge pins to output
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enable_pin_a, OUTPUT);
  pinMode(enable_pin_b, OUTPUT);
  

    // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  Serial.println("Mac address for the wrom wroom: " + WiFi.macAddress());

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("added to peer");
  }
  
    // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("x: ");
  Serial.println(incoming_x);
  Serial.println("y: ");
  Serial.println(incoming_y);
//  Serial.println("y: " + incoming_y);
  delay(100);

  analogWrite(enable_pin_a, 200);
  analogWrite(enable_pin_b, 200);

  if (incoming_x > -3 && incoming_x < 4 && incoming_y > -3 && incoming_y < 4) {
    analogWrite(enable_pin_a, 0);
    analogWrite(enable_pin_b, 0);

    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }
  if(incoming_x > 4) {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);

    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
  }
  if (incoming_y > 4) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);

    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
  }
  if (incoming_x < -3) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  }
  if (incoming_y < -3) {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);

    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  }
  delay(10);
}
