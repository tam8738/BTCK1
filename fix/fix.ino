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
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>
#include "EEPROM.h"

#define PIN_BUZZER 20
#define PIN_RELAY_PUMP 42
#define PIN_RELAY_LIGHT 40
#define PIN_RELAY_VAN_1 47
#define PIN_RELAY_VAN_2 21
bool isFirstTimeDevice = true;

const char* ssid = "Tam";
const char* password = "12345678";

Adafruit_AHTX0 aht;
int temperature = 0;
int humidity = 0;
int soilMoisturePercentage = 0;
int LDRPercentage = 0;
bool automode = true;
int Soil1 = 0;
int Soil2 = 0;

bool valveState1 = false;
bool valveState2 = false;
bool pumpManualState = false;
bool lightManualState = false;

// Cấu hình thời gian thực
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 7*3600; // GMT+7 cho Việt Nam
const int daylightOffset_sec = 0;
const char *time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";

// Biến thời gian
String currentTime = "";
String currentDate = "";
unsigned long lastTimeUpdate = 0;
bool timeInitialized = false;

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
    int a;
    int b;
} struct_message;

struct_message myData;

int currentScreen = 0;
uint8_t setupSoilMoisteur = 50;
uint8_t setupLightSensor = 30;

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
const char* topicValve1 = "iot/control/valve1";
const char* topicValve2 = "iot/control/valve2";
const char* topicThresholds = "iot/control/thresholds";

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
      digitalWrite(PIN_RELAY_PUMP, HIGH); 
      pumpManualState = true;
      automode = false;
      sendDataMQTT();
      break;
    case '3': // Tắt bơm
      lv_obj_clear_state(ui_bom, LV_STATE_CHECKED);
      digitalWrite(PIN_RELAY_PUMP, LOW); 
      pumpManualState = false;
      automode = false;
      sendDataMQTT();
      break;
    case '4': // Bật đèn
      lv_obj_add_state(ui_den, LV_STATE_CHECKED);
      digitalWrite(PIN_RELAY_LIGHT, HIGH);
      lightManualState = true;
      automode = false;
      sendDataMQTT();
      break;
    case '5': // Tắt đèn
      lv_obj_clear_state(ui_den, LV_STATE_CHECKED);
      digitalWrite(PIN_RELAY_LIGHT, LOW);
      lightManualState = false;
      automode = false;
      sendDataMQTT();
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
  sendDataMQTT();
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

void setup(){
  Serial.begin(115200);
  EEPROM.begin(64);

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

  // Khởi tạo AHT10 sensor
  if (!aht.begin()) {
    Serial.println("Could not find AHT10? Check wiring");
  }

  // Cài đặt pin mode
  pinMode(PIN_RELAY_PUMP, OUTPUT); // relay đèn
  pinMode(PIN_RELAY_LIGHT, OUTPUT); // relay bơm
  pinMode(PIN_RELAY_VAN_1, OUTPUT); // relay van 1
  pinMode(PIN_RELAY_VAN_2, OUTPUT); // relay van 2
  pinMode(PIN_BUZZER, OUTPUT); // còi
  pinMode(5, INPUT_PULLUP); // công tắc phao
  digitalWrite(PIN_RELAY_LIGHT, LOW); // Tắt relay đèn
  digitalWrite(PIN_RELAY_PUMP, LOW); // Tắt relay bơm
  digitalWrite(PIN_RELAY_VAN_1, HIGH); // Tắt relay van 1
  digitalWrite(PIN_RELAY_VAN_2, HIGH); // Tắt relay van 2
  digitalWrite(PIN_BUZZER, LOW); // Tắt còi

  // Kết nối WiFi
  WiFi.mode(WIFI_AP_STA);
  setup_wifi();
  
  // Cài đặt MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  // Cài đặt SSL certificate cho WiFiClientSecure
  espClient.setCACert(root_ca);

  // Cấu hình thời gian thực
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    static unsigned long lastSensorRead = 0;
    static unsigned long lastControlCheck = 0;
    static unsigned long lastMQTTProcess = 0;
    
    lv_timer_handler();
    
    // Cập nhật thời gian mỗi giây
    if (millis() - lastTimeUpdate >= 1000) {
        updateTime();
        lastTimeUpdate = millis();
    }
    // Đọc cảm biến mỗi 2 giây
    if (millis() - lastSensorRead >= 100) {
      readAHT10();
      readLDR();
      updateDisplay();
      lastSensorRead = millis();
    }
    // Kiểm tra điều khiển mỗi 1 giây
  if (millis() - lastControlCheck >= 100) {
    controlPump();
    controlLight();
    lastControlCheck = millis();
  }
  // Xử lý MQTT mỗi 100ms
  if (millis() - lastMQTTProcess >= 100) {
    mqttClient.loop();
    lastMQTTProcess = millis();
  }
    
  delay(5);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("From: ");
  // Serial.println(myData.a);
  // Serial.print("Value: ");
  // Serial.println(myData.b);
  // Serial.println();

  if (myData.a == 1){
    Serial.print(" soil 1 : ");
    Soil1 = myData.b;
    Serial.println(myData.b);
  }
  if (myData.a == 2){
    Serial.print(" soil 2 : ");
    Soil2 = myData.b;
    Serial.println(myData.b);
  }
}

void readAHT10(){
  sensors_event_t humidity_event, temp_event;
  aht.getEvent(&humidity_event, &temp_event);
  temperature = temp_event.temperature;
  humidity = humidity_event.relative_humidity;
  lv_label_set_text_fmt(ui_nhietDo_txt, "Nhiet do: %d °C", temperature);
  lv_label_set_text_fmt(ui_doAm_txt, "Do am: %d %%", humidity);
  lv_bar_set_value(ui_doAmHT, 50, LV_ANIM_OFF);
  lv_bar_set_value(ui_nhietDoHT, temperature, LV_ANIM_OFF);  // cập nhật thanh bar nhiệt độ
  sendDataMQTT();
}

void readLDR(){
  int raw = analogRead(4);
  LDRPercentage = map(raw, 4095, 0, 0, 100);
  LDRPercentage = constrain(LDRPercentage, 0, 100);
  lv_label_set_text_fmt(ui_anhSang_txt, "Anh sang: %d %%", LDRPercentage);
  lv_bar_set_value(ui_anhSangHT, LDRPercentage, LV_ANIM_OFF); // cập nhật thanh bar ánh sáng
  sendDataMQTT();
}

bool checkWater() {
    // Đọc trạng thái công tắc phao ở chân 5, LOW là có nước, HIGH là hết nước (chỉnh lại nếu cần)
    if (analogRead(5)<4080){
      return false;
    }
    else{
      return true;
    }
}

void getLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        //Serial.println("Failed to obtain time");
        return;
    }
    
    // Format thời gian HH:MM
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    currentTime = String(timeStr);
    
    timeInitialized = true;
    //Serial.println("Time updated: " + currentTime);
}

void updateTime() {
    // Cập nhật thời gian mỗi giây
    if (millis() - lastTimeUpdate >= 1000) {
        getLocalTime();
        lastTimeUpdate = millis();
    }
    
    // Hiển thị thời gian lên màn hình
    if (timeInitialized) {
        lv_label_set_text(ui_time, currentTime.c_str());
    } else {
        lv_label_set_text(ui_time, "No Time");
    }
}

void updateDisplay(){
  lv_label_set_text_fmt(ui_nhietDo_txt, "Nhiet do: %d C", temperature);
  lv_label_set_text_fmt(ui_doAm_txt, "Do am: %d %%", humidity);
  lv_label_set_text_fmt(ui_anhSang_txt, "Anh sang: %d %%", LDRPercentage);
  updateTime(); // Cập nhật thời gian
}

void controlPump() {
  static unsigned long buzzerStartTime = 0;
  static bool buzzerOn = false;
  bool water = checkWater();
  //Kiểm tra nước, nếu hết nước bật còi cảnh báo
  if (water == false) {
    Serial.println("no water!");
    digitalWrite(PIN_RELAY_PUMP, LOW);                      // Tắt bơm
    digitalWrite(PIN_RELAY_VAN_1, HIGH);                      // Tắt van 1
    digitalWrite(PIN_RELAY_VAN_2, HIGH);                      // Tắt van 2
    lv_obj_clear_state(ui_bom, LV_STATE_CHECKED); // Update giao diện
    lv_label_set_text(ui_batbom, "het nuoc");
    sendDataMQTT();
    if (buzzerOn == false) {
      //digitalWrite(PIN_BUZZER, HIGH);           // Bật còi
      tone(PIN_BUZZER, 2000);
      buzzerOn = true;
      buzzerStartTime = millis();
    }
  }
  //nếu còn nước thì bơm và bật van nước
  else {
    //Serial.println("ready to pump!");
    lv_label_set_text(ui_batbom, "bat bom");
    // check còi => turn off
    if (buzzerOn == true) {
      tone(PIN_BUZZER, 0);
      buzzerOn = false;
    }
    // Chế độ tự động
    if (automode) {
      //Serial.println("automode: on");
      // bật van nước
      Serial.print("SOIL1:");
      Serial.println(Soil1);
      Serial.print("SOIL2:");
      Serial.println(Soil2);
      if (Soil1 < setupSoilMoisteur || Soil2 < setupSoilMoisteur) {
        digitalWrite(PIN_RELAY_PUMP, HIGH);                   // Bật bơm
        lv_obj_add_state(ui_bom, LV_STATE_CHECKED);
        if (Soil1 < setupSoilMoisteur){
          //Serial.println("bật van 1");
          digitalWrite(PIN_RELAY_VAN_1, LOW);
        }
        if (Soil2 < setupSoilMoisteur){
          //Serial.println("bật van 2");
          digitalWrite(PIN_RELAY_VAN_2, LOW);
        }
        if (Soil1 > setupSoilMoisteur){
          //Serial.println("tắt van 1");
          digitalWrite(PIN_RELAY_VAN_1, HIGH);
        }
        if (Soil2 > setupSoilMoisteur){
          //Serial.println("tắt van 2");
          digitalWrite(PIN_RELAY_VAN_2, HIGH);
        }
        sendDataMQTT();
      }
      else{
        Serial.println("all off");
        digitalWrite(PIN_RELAY_PUMP, LOW);
        digitalWrite(PIN_RELAY_VAN_1, HIGH);
        digitalWrite(PIN_RELAY_VAN_2, HIGH);
        sendDataMQTT();
      }
    }
    // chế độ thủ công
    else {
      //Serial.println("automode: off");
      if (pumpManualState) {
        if(valveState1){
          digitalWrite(PIN_RELAY_VAN_1, LOW);                   // Bật van 1
        }
        else{
          digitalWrite(PIN_RELAY_VAN_1, HIGH);
        }
        if(valveState2){
          digitalWrite(PIN_RELAY_VAN_2, LOW); 
        }
        else{
          digitalWrite(PIN_RELAY_VAN_2, HIGH);  
        }
        sendDataMQTT();
      }
    }
  }
}

void controlLight() {
  if (automode) {
    if (LDRPercentage < setupLightSensor) {
      // Trời tối, bật đèn
      digitalWrite(PIN_RELAY_LIGHT, HIGH); // Bật relay đèn
      lv_obj_add_state(ui_den, LV_STATE_CHECKED);
      sendDataMQTT();
    } 
    else {
      // Đủ sáng, tắt đèn
      digitalWrite(PIN_RELAY_LIGHT, LOW); // Tắt relay đèn
      lv_obj_clear_state(ui_den, LV_STATE_CHECKED);
      sendDataMQTT();
    }
  }
  else {
    // Thủ công: điều khiển đèn bằng nút hoặc lệnh ngoài (biến lightManualState)
    if (lightManualState) {
      digitalWrite(PIN_RELAY_LIGHT, HIGH); // Bật đèn
      lv_obj_add_state(ui_den, LV_STATE_CHECKED);
      sendDataMQTT();
    }
    else {
      digitalWrite(PIN_RELAY_LIGHT, LOW); // Tắt đèn
      lv_obj_clear_state(ui_den, LV_STATE_CHECKED);
      sendDataMQTT();
      }
  }
}

void setup_wifi(){
  Serial.println("Setting up WiFi...");
  
  if (isFirstTimeDevice == true){
    Serial.println("Starting SmartConfig...");
    WiFi.beginSmartConfig();
    // Chờ SmartConfig với timeout 60 giây
    int timeout = 0;
    while (!WiFi.smartConfigDone() && timeout < 120) { 
      delay(500); 
      Serial.print(".");
      timeout++;
    }
    
    if (WiFi.smartConfigDone()) {
      Serial.println("\nSmartConfig done!");
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("Password: ");
      Serial.println(WiFi.psk());
      // write SSID and password to eeprom
      EEPROM.writeString(0, WiFi.SSID());
      EEPROM.writeString(1, WiFi.psk());
    } else {
      Serial.println("\nSmartConfig timeout!");
    }
    isFirstTimeDevice = false;
  }
  else {
    String ssid = EEPROM.readString(0);
    String password = EEPROM.readString(1);
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Pass: ");
    Serial.println(password);
    WiFi.begin(ssid, password);
  }
  // Chờ kết nối WiFi
  Serial.println("Connecting to WiFi...");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 60) { 
    delay(500); 
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Lấy thời gian sau khi kết nối WiFi
    getLocalTime();
  } else {
    Serial.println("\nWiFi connection failed!");
    isFirstTimeDevice = true;
  }
}

void reconnectMQTT(){
  while (!mqttClient.connected()) {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      mqttClient.subscribe(topicPump);
      mqttClient.subscribe(topicLight);
      mqttClient.subscribe(topicMode);
      mqttClient.subscribe(topicValve1);
      mqttClient.subscribe(topicValve2);
      mqttClient.subscribe(topicThresholds);
    } else {
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);
  
  if (String(topic) == topicPump) {
    if (message == "ON") {
      pumpManualState = true;
      automode = false;
      digitalWrite(PIN_RELAY_PUMP, HIGH);
      lv_obj_add_state(ui_bom, LV_STATE_CHECKED);
    } else if (message == "OFF") {
      pumpManualState = false;
      automode = false;
      digitalWrite(PIN_RELAY_PUMP, LOW);
      lv_obj_clear_state(ui_bom, LV_STATE_CHECKED);
    }
  } else if (String(topic) == topicLight) {
    if (message == "ON") {
      lightManualState = true;
      automode = false;
      digitalWrite(PIN_RELAY_LIGHT, HIGH);
      lv_obj_add_state(ui_den, LV_STATE_CHECKED);
    } else if (message == "OFF") {
      lightManualState = false;
      automode = false;
      digitalWrite(PIN_RELAY_LIGHT, LOW);
      lv_obj_clear_state(ui_den, LV_STATE_CHECKED);
    }
  } else if (String(topic) == topicMode) {
    if (message == "AUTO" || message == "1") {
      automode = true;
    } else if (message == "MANUAL" || message == "0") {
      automode = false;
    }
  } else if (String(topic) == topicValve1) {
    if (message == "ON") {
      valveState1 = true;
      digitalWrite(PIN_RELAY_VAN_1, LOW);
    } else if (message == "OFF") {
      valveState1 = false;
      digitalWrite(PIN_RELAY_VAN_1, HIGH);
    }
  } else if (String(topic) == topicValve2) {
    if (message == "ON") {
      valveState2 = true;
      digitalWrite(PIN_RELAY_VAN_2, LOW);
    } else if (message == "OFF") {
      valveState2 = false;
      digitalWrite(PIN_RELAY_VAN_2, HIGH);
    }
  } else if (String(topic) == topicThresholds) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (!error) {
      if (doc.containsKey("setupSoilMoisteur")) {
        setupSoilMoisteur = doc["setupSoilMoisteur"];
        lv_label_set_text_fmt(ui_doAmSetUp, "Do am dat: %d %%", setupSoilMoisteur);
        lv_slider_set_value(ui_SetupAm, setupSoilMoisteur, LV_ANIM_OFF);
      }
      if (doc.containsKey("setupLightSensor")) {
        setupLightSensor = doc["setupLightSensor"];
        lv_label_set_text_fmt(ui_anhSangSetup, "Anh sang: %d %%", setupLightSensor);
        lv_slider_set_value(ui_Setupsang, setupLightSensor, LV_ANIM_OFF);
      }
    }
  }
  sendDataMQTT();
}

void sendDataMQTT() {
  if (!mqttClient.connected()) reconnectMQTT();
  StaticJsonDocument<512> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["soil1"] = Soil1;
  doc["soil2"] = Soil2;
  doc["light"] = LDRPercentage;
  doc["water_level"] = checkWater() ? "available" : "low";
  doc["pump_state"] = (digitalRead(PIN_RELAY_PUMP) == HIGH) ? "ON" : "OFF";
  doc["light_state"] = (digitalRead(PIN_RELAY_LIGHT) == HIGH) ? "ON" : "OFF";
  doc["valve1_state"] = (digitalRead(PIN_RELAY_VAN_1) == LOW) ? "ON" : "OFF";
  doc["valve2_state"] = (digitalRead(PIN_RELAY_VAN_2) == LOW) ? "ON" : "OFF";
  doc["automode"] = automode;
  doc["setupSoilMoisteur"] = setupSoilMoisteur;
  doc["setupLightSensor"] = setupLightSensor;
  
  char buffer[512];
  serializeJson(doc, buffer);
  mqttClient.publish(topicData, buffer);
}