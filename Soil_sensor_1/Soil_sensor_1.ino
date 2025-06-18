#include <esp_now.h>
#include <WiFi.h>
const int DryValue = 3130;   //you need to replace this value with Value_1 
const int WaterValue = 1645;  //you need to replace this value with Value_2 
int soilMoistureValue = 0; 
int soilMoisturePercent = 0;

uint8_t broadcastAddress[] = {0x30, 0x30, 0xF9, 0x00, 0x3B, 0x64};
typedef struct struct_message {
  char a[32];
  int b;
} struct_message;

struct_message myData;

String success;
int count = 0;

esp_now_peer_info_t peerInfo;


void setup() { 
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
} 
void loop() { 
  soilMoistureValue = analogRead(4);  //put Sensor insert into soil 
  soilMoisturePercent = map(soilMoistureValue,1800,4095,100,0);
  constrain(soilMoistureValue, 0, 100);
  Serial.print("Soil Moisture Percent: ");
  Serial.println(soilMoisturePercent);
  delay(100);
  // Send message via ESP-NOW
  strcpy(myData.a, "SOIL 1");
  myData.b = soilMoisturePercent;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10000);
} 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Bytes received: ");
  Serial.println(len);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}