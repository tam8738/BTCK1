#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <Keypad.h>
#include <esp_now.h>
#include <WiFi.h>

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
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  Serial.print("From: ");
  Serial.println(myData.a);
  Serial.print("Value: ");
  Serial.println(myData.b);
  Serial.println();
}