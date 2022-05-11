#include <WiFi.h>
#include <esp_now.h>

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
  Serial.print("Bytes received: ");
  Serial.println(len);
 
  incoming_x = message.x;
  incoming_y = message.y;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

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
  Serial.print("test");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("x: " + incoming_x);
//  Serial.print();
//  Serial.println("y: " + incoming_y);
  delay(100);
  
}
