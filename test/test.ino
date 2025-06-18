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

// ----------- TFT/LVGL CONFIG -----------
static const uint16_t screenWidth  = 160;
static const uint16_t screenHeight = 128;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

// ----------- KEYPAD CONFIG -------------
const byte ROWS = 1, COLS = 5;
char hexaKeys[ROWS][COLS] = { {'1','2','3','4','5'} };
byte rowPins[ROWS] = {48};
byte colPins[COLS] = {35,36,37,38,39};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// ----------- SENSOR & GLOBAL VARS -----------
TwoWire I2C_AHT = TwoWire(0);
Adafruit_AHTX0 aht;
float temperature = 0;
float humidity = 0;
int soilMoisturePercentage = 0;
int LDRPercentage = 0;
int mucNuoc = 0; // ADC raw

const int relayPump = 40;
const int relayLight = 42;
bool autoMode = true;
int currentScreen = 0;
uint8_t setupSoilMoisteur = 30;
uint8_t setupLightSensor = 30;

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

// ----------- WIFI SMARTCONFIG -----------
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.beginSmartConfig();
  while (!WiFi.smartConfigDone()) { delay(500); }
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
}

// ----------- LVGL FLUSH -----------
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p ) {
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );
  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
  tft.endWrite();
  lv_disp_flush_ready( disp );
}

// ----------- SENSOR FUNCTIONS -----------
void readAHT10() {
  sensors_event_t hum, temp;
  if (aht.getEvent(&hum, &temp)) {
    temperature = temp.temperature;
    humidity = hum.relative_humidity;
  }
}
void readSoilMoisture() {
  int raw = analogRead(5);
  soilMoisturePercentage = map(raw, 0, 4095, 0, 100);
  soilMoisturePercentage = constrain(soilMoisturePercentage, 0, 100);
}
void readLDR() {
  int raw = analogRead(4);
  LDRPercentage = map(raw, 4095, 0, 0, 100);
  LDRPercentage = constrain(LDRPercentage, 0, 100);
}
void checkWaterLevel() {
  mucNuoc = analogRead(6);
}

// ----------- DEVICE CONTROL -----------
void controlPump(bool on) {
  if (on && mucNuoc > 100) {
    // Không bật nếu hết nước
    return;
  }
  digitalWrite(relayPump, on ? LOW : HIGH); // Active LOW
  // Update LVGL state for pump
  if (on) lv_obj_add_state(ui_bom, LV_STATE_CHECKED);
  else lv_obj_clear_state(ui_bom, LV_STATE_CHECKED);
}
void controlLight(bool on) {
  digitalWrite(relayLight, on ? LOW : HIGH); // Active LOW
  if (on) lv_obj_add_state(ui_den, LV_STATE_CHECKED);
  else lv_obj_clear_state(ui_den, LV_STATE_CHECKED);
}

// ----------- AUTO CONTROL LOGIC -----------
void automaticControl() {
  if(autoMode) {
    bool pumpState = (soilMoisturePercentage < setupSoilMoisteur) && (mucNuoc <= 100);
    bool lightState = (LDRPercentage < setupLightSensor);
    controlPump(pumpState);
    controlLight(lightState);
  }
}

// ----------- MQTT FUNCTIONS -----------
void reconnectMQTT() {
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
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];
  if (String(topic) == topicMode)
    autoMode = (message == "AUTO");
  // MQTT chỉ tác dụng khi autoMode, ưu tiên nút vật lý
  if (autoMode) {
    if (String(topic) == topicPump) {
      controlPump(message == "ON");
    }
    if (String(topic) == topicLight) {
      controlLight(message == "ON");
    }
  }
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

// ----------- BUTTON HANDLERS -----------
void hienThiBtn(char key) {
  switch(key) { 
    case '1': // Chuyển sang màn hình điều khiển
      _ui_screen_change(&ui_dieuKhien, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, &ui_dieuKhien_screen_init);
      currentScreen = 1;
      break;
    default:
      break;
  }
}
void dieuKhienBtn(char key) {
  switch(key) { 
    case '2': // Bật bơm
      controlPump(true);
      mqttClient.publish(topicPump, "ON");
      break;
    case '3': // Tắt bơm
      controlPump(false);
      mqttClient.publish(topicPump, "OFF");
      break;
    case '4': // Bật đèn
      controlLight(true);
      mqttClient.publish(topicLight, "ON");
      break;
    case '5': // Tắt đèn
      controlLight(false);
      mqttClient.publish(topicLight, "OFF");
      break;
    case '1': // Chuyển sang màn hình setup
      _ui_screen_change(&ui_setUp, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, &ui_setUp_screen_init);
      currentScreen = 2;
      break;
    default:
      break;
  }
}
void setupBtn(char key) {
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

// ----------- KEYPAD TO LVGL INPUT -----------
void my_keypad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data ){
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

// ----------- UPDATE DISPLAY -----------
void updateDisplay() {
  lv_label_set_text_fmt(ui_nhietDo_txt, "nhiet do: %5.1f C", temperature);
  lv_label_set_text_fmt(ui_doAm_txt, "do am: %5.1f %%", humidity);
  lv_label_set_text_fmt(ui_anhSang_txt, "anh sang: %d %%", LDRPercentage);
  lv_label_set_text_fmt(ui_doAmHT, "do am dat: %d %%", soilMoisturePercentage);
}

// ----------- SETUP -----------
void setup() {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(3);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_keypad_read;
  lv_indev_drv_register(&indev_drv);
  ui_init();

  // Khởi tạo cảm biến, relay
  I2C_AHT.begin(8, 9);
  aht.begin(&I2C_AHT, 0x38);
  pinMode(relayPump, OUTPUT); pinMode(relayLight, OUTPUT);
  digitalWrite(relayPump, HIGH); digitalWrite(relayLight, HIGH);
  pinMode(5, INPUT); pinMode(6, INPUT); pinMode(4, INPUT);

  // WiFi + MQTT
  setup_wifi();
  espClient.setCACert(root_ca);
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
}

// ----------- LOOP -----------
void loop() {
  static uint32_t lastUpdate = 0, lastMQTT = 0;
  lv_timer_handler();
  if (millis() - lastUpdate > 1000) {
    readAHT10();
    readSoilMoisture();
    readLDR();
    checkWaterLevel();
    automaticControl();
    updateDisplay();
    lastUpdate = millis();
  }
  if (millis() - lastMQTT > 2000) {
    sendDataMQTT();
    lastMQTT = millis();
  }
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();
  delay(5);
}