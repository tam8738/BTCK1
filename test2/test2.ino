// cảm biến ánh sáng: analog(4)
// relay đèn: 40
// relay bơm: 42
// aht10: SDA(8), SCL(9)
// công tắc phao: 5
// 2 relay van nước: 47,20.
// module nút nhấn: 35,36,37,38,39/48

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <Keypad.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

Adafruit_AHTX0 aht;
int temperature = 0;
int humidity = 0;
int soilMoisturePercentage = 0;
int LDRPercentage = 0;
int mucNuoc = 0; // ADC raw

/* Change to your screen resolution */
static const uint16_t screenWidth  = 160;
static const uint16_t screenHeight = 128;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

// Keypad setup
const byte ROWS = 1;
const byte COLS = 5;
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','4','5'},
};
byte rowPins[ROWS] = {48}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {35,36,37,38,39}; // connect to the column pinouts of the keypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

typedef struct struct_message {
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

struct_message myData;


int currentScreen = 0;
uint8_t setupSoilMoisteur = 0;
uint8_t setupLightSensor = 0;

#if LV_USE_LOG != 0
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}
// ----------- MQTT CONFIG -----------
const char* mqttServer = "a7e103b17493474fb922fbc4a7a81412.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "Tuoi_cay";
const char* mqttPassword = "Tuoi_cay1";
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
const char* topicData   = "iot/data";
const char* topicPump   = "iot/control/pump";
const char* topicLight  = "iot/control/light";
const char* topicMode   = "iot/control/mode";
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";
// --------- Button handlers ----------
void hienThiBtn (char key) {
  switch(key) { 
    case '1': // Chuyển sang màn hình điều khiển
      _ui_screen_change(&ui_dieuKhien, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, &ui_dieuKhien_screen_init);
      currentScreen = 1;
      break;
    default:
      break;
  }
}

void dieuKhienBtn (char key) {
  switch(key) { 
    case '2': // Bật bơm
      lv_obj_add_state(ui_bom, LV_STATE_CHECKED);
      break;
    case '3': // Tắt bơm
      lv_obj_clear_state(ui_bom, LV_STATE_CHECKED);
      break;
    case '4': // Bật đèn
      lv_obj_add_state(ui_den, LV_STATE_CHECKED);
      break;
    case '5': // Tắt đèn
      lv_obj_clear_state(ui_den, LV_STATE_CHECKED);
      break;
    case '1': // Chuyển sang màn hình setup
      _ui_screen_change(&ui_setUp, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, &ui_setUp_screen_init);
      currentScreen = 2;
      break;
    default:
      break;
  }
}

void setupBtn (char key) {
  switch(key) { 
    case '2': // Tăng độ ẩm
      setupSoilMoisteur++;
      setupSoilMoisteur = constrain(setupSoilMoisteur, 0, 100);
      break;
    case '3': // Giảm độ ẩm
      setupSoilMoisteur--;
      setupSoilMoisteur = constrain(setupSoilMoisteur, 0, 100);
      break;
    case '4': // Tăng ánh sáng
      setupLightSensor++;
      setupLightSensor = constrain(setupLightSensor, 0, 100);
      break;
    case '5': // Giảm ánh sáng
      setupLightSensor--;
      setupLightSensor = constrain(setupLightSensor, 0, 100);
      break;
    case '1': // Quay lại màn hình hiển thị
      _ui_screen_change(&ui_hienthi, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, &ui_hienthi_screen_init);
      currentScreen = 0;
      break;
    default:
      break;
  }
  // Cập nhật giao diện
  lv_label_set_text_fmt(ui_doAmSetUp, "Do am dat: %d %%", setupSoilMoisteur);
  lv_label_set_text_fmt(ui_anhSangSetup, "Anh sang: %d %%", setupLightSensor);
  lv_slider_set_value(ui_SetupAm, setupSoilMoisteur, LV_ANIM_OFF);
  lv_slider_set_value(ui_Setupsang, setupLightSensor, LV_ANIM_OFF);
}

// --------- Keypad to LVGL input ----------
void my_keypad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data ){
  static char lastKey = 0;
  char customKey = customKeypad.getKey();
  if (customKey && customKey != lastKey){
    lastKey = customKey;
    if(currentScreen == 0 ) hienThiBtn(customKey);
    else if(currentScreen == 1 ) dieuKhienBtn(customKey);
    else if(currentScreen == 2 ) setupBtn(customKey);
    delay(100);
  }
  if (!customKey) lastKey = 0;
}

void setup()
{
    Serial.begin(115200);

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
#endif
  tft.begin();
  tft.setRotation(3);

  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_keypad_read;
  lv_indev_drv_register( &indev_drv );

  ui_init();

  Serial.println( "Setup done" );

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{
    lv_timer_handler();
    delay(5);
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("From: ");
  Serial.println(myData.a);
  Serial.print("Value: ");
  Serial.println(myData.b);
  Serial.println();

  if (strcmp(myData.a, "ESP1") == 0) {
      // Giá trị độ ẩm đất
      Soil1 = myData.b;
  } else if (strcmp(myData.a, "ESP2") == 0) {
      Soil2 = myData.b;
  }
}
void readAHT10(){
  sensors_event_t humidity_event, temp_event;
  aht.getEvent(&humidity_event, &temp_event);
  temperature = temp_event.temperature;
  humidity = humidity_event.relative_humidity;
  lv_label_set_text_fmt(ui_nhietDo_txt, "Nhiet do: %d °C", temperature);
  lv_label_set_text_fmt(ui_doAm_txt, "Do am: %d %%", humidity);
  // Đã bỏ lv_bar_set_value(ui_nhietDo1, ...), ui_doAm1 vì không có trong ui.h
}

void readLDR(){
  int raw = analogRead(4);
  LDRPercentage = map(raw, 4095, 0, 0, 100);
  LDRPercentage = constrain(LDRPercentage, 0, 100);
  lv_label_set_text_fmt(ui_anhSang_txt, "Anh sang: %d %%", LDRPercentage);
  // Đã bỏ lv_bar_set_value(ui_anhSang1, ...) vì không có trong ui.h
}
void checkWater(){

}
void updateDisplay(){
  lv_label_set_text_fmt(ui_nhietDo_txt, "Nhiet do: %d C", temperature);
  lv_label_set_text_fmt(ui_doAm_txt, "Do am: %d %%", humidity);
  lv_label_set_text_fmt(ui_anhSang_txt, "Anh sang: %d %%", LDRPercentage);
  lv_label_set_text_fmt(ui_doAmHT, "Do am dat: %d %%", soilMoisturePercentage);
}
void control(){

}
void setup_wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.beginSmartConfig();
  while (!WiFi.smartConfigDone()) { delay(500); }
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
}
void reconnectMQTT(){
  while (!mqttClient.connected()) {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      mqttClient.subscribe(topicPump);
      mqttClient.subscribe(topicLight);
      mqttClient.subscribe(topicMode);
    } else {
      delay(5000);
    }
  }
}
void mqttCallback(char* topic, byte* payload, unsigned int length) {

}
void sendDataMQTT() {
  if (!mqttClient.connected()) reconnectMQTT();
  StaticJsonDocument<256> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["soil_moisture"] = soilMoisturePercentage;
  doc["light"] = LDRPercentage;
  doc["water_level"] = (mucNuoc <= 100) ? "available" : "low";
  doc["pump_state"] = (digitalRead(relayPump) == LOW) ? "ON" : "OFF";
  doc["light_state"] = (digitalRead(relayLight) == LOW) ? "ON" : "OFF";
  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(topicData, buffer);
}