#include <SimpleFOC.h>
#include <Wire.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include "RGB.h"
#include "BleKeyboard.h"
#include "ui.h"

BleKeyboard bleKeyboard;
//设置引脚
/*
   RST：9
   DC：14
   SDA(MOSI)：11
   BLK：13
   SCL(SCK)：12
   CS：10
   GND：GND
   VCC：3.3V
*/
#define OFF_PIN 7
#define OFF_UP_PIN 6
#define PUSH_BUTTON 5
uint8_t push_flag, push_states;
uint32_t push_time, push_in_time, push_two_time;
uint8_t dial_flag;
#define SCLK 12
#define MOSI 11
#define TFT_CS 10
#define TFT_BLK 13
#define TFT_DC 14
#define TFT_RST 9
#define MO1 17
#define MO2 16
#define MO3 15
#define MT6701_SDA 1
#define MT6701_SCL 2

// 使用软SPI
// Arduino_DataBus *bus = new Arduino_SWSPI(TFT_DC, TFT_CS, 18 /* SCK */, 23 /* MOSI */, -1 /* MISO */);

// 使用硬 SPI
// Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);

// 自定义引脚
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, SCLK, MOSI, MISO);

// 使用GC9A01 IPS LCD 240x240
Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, TFT_RST, 0 /* 屏幕方向 */, true /* IPS */);
/*******************************************************************************
   End of Arduino_GFX setting
 ******************************************************************************/

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
uint8_t lv_page = 0;
uint16_t img_angle = 0;
int16_t dial_angle = 30;
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);

  lv_disp_flush_ready(disp);
}

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(MO1, MO2, MO3);
//目标变量
float target_velocity = 0;
float row_angle;

float readMySensorCallback() {
  uint8_t i2cData[2];  // Buffer for I2C data
  i2cRead(0x03, i2cData, 1);
  i2cRead(0x04, i2cData + 1, 1);
  uint16_t ag = i2cData[0] << 8 | i2cData[1];
  ag = ag >> 2;
  float rad = (float)ag * 2 * PI / 16384;
  if (rad < 0) {
    rad += 2 * PI;
  }
  return rad;
}

void initMySensorCallback() {
  // do the init
  Wire.begin(MT6701_SDA, MT6701_SCL, uint32_t(400000));  // Set I2C frequency to 400kHz
}

// create the sensor
GenericSensor sensor = GenericSensor(readMySensorCallback, initMySensorCallback);

void setup() {
  // monitoring port
  Serial.begin(115200);
  delay(100);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  pinMode(OFF_UP_PIN,OUTPUT);
  pinMode(OFF_PIN,OUTPUT);
  digitalWrite(OFF_PIN,LOW);
  digitalWrite(OFF_UP_PIN,LOW);
  strip.begin();                    // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();                     // Turn OFF all pixels ASAP
  strip.setBrightness(brightness);  // Set BRIGHTNESS to about 1/5 (max = 255)
  // initialize sensor hardware
  sensor.init();
  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);
  //供电电压设置 [V]
  driver.voltage_power_supply = 5;
  driver.init();
  motor.linkDriver(&driver);
  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //运动控制模式设置
  motor.controller = MotionControlType::torque;
  //速度PI环设置
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 15;
  //最大电机限制电机
  motor.voltage_limit = 5;
  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;
  //设置最大速度限制
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);
  //初始化电机
  motor.init();
  //初始化 FOC
  // float zero_electric_offset = 1.62;
  // Direction foc_direction = Direction::CCW;
  // motor.initFOC(zero_electric_offset, foc_direction);
 motor.initFOC();
  Serial.println("Starting BLE work!");
  bleKeyboard.begin();
  display_init();
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  row_angle = motor.shaft_angle;
}

void loop() {
  unsigned long currentMillis = millis();
  //rgb显示
  if (currentMillis - pixelPrevious >= pixelInterval) {  //  Check for expired time
    pixelPrevious = currentMillis;                       //  Run current frame
    rainbow2();
    switch (rgb_flag) {
      case 0:
        break;
      case 1:
        role_fill();
        break;
      case 2:
        rainbow2();
        break;
    }
  }
  //----------------------旋转部分
  float now_angle = motor.shaft_angle - row_angle;
  if (abs(now_angle) > dial_angle * 0.005) {
    //转动一度刷新指针位置
    row_angle = motor.shaft_angle;
    if (now_angle > 0)
      dial_flag = 2;  //左旋
    else
      dial_flag = 1;  //右旋
  }
  if (dial_flag) {
    switch (lv_page) {
      case 0:  //页面0
        if (dial_flag == 1) {
          if (img_angle <= 0)
            img_angle = 3600;
          img_angle = img_angle - 30;
          strip2();
        } else {
          if (img_angle >= 3600)
            img_angle = 0;
          img_angle = img_angle + 30;
          strip3();
        }
        //如果蓝牙连接成功
        if (bleKeyboard.isConnected()) {
          if (dial_flag == 1) {
            Serial.println("Sending L");
            bleKeyboard.sendDialReport(DIAL_L);
          } else {
            Serial.println("Sending R");
            bleKeyboard.sendDialReport(DIAL_R);
          }
        }
        break;
      case 1:
        if (dial_flag == 1) {
          dial_angle = dial_angle - 1;
          if (dial_angle <= 0)
            dial_angle = 0;
          strip2();
        } else {
          dial_angle = dial_angle + 1;
          if (dial_angle >= 100)
            dial_angle = 100;
          strip3();
        }
        lv_meter_set_indicator_value(meter, line_indic, dial_angle);
    }

    dial_flag = 0;
    lv_img_set_angle(ui_Image2, img_angle);
  }
  //------------------------按键处理----------------------//
  if (digitalRead(PUSH_BUTTON) == 0) {  //按下
    if (push_flag == 0) {
      push_time = currentMillis;
      push_flag = 1;
    }
    if (currentMillis - push_time > 10) {  //消抖10ms
      if (push_flag == 1) {
        push_flag = 2;
        push_in_time = currentMillis;
        if (push_in_time - push_two_time < 500)
          push_states = 2;
        else
          push_states = 1;
        Serial.println("DIAL_PRESS");
      }
    }
    if (push_flag == 2) {  //长按
      if (currentMillis - push_in_time > 300) {
        push_states = 3;
      }
    }
  }
  if (push_flag && digitalRead(PUSH_BUTTON)) {  //松开
    push_two_time = currentMillis;
    push_flag = 0;
    push_states = 4;
  }
  if (push_states) {  //按钮状态检测，单击双击
    switch (push_states) {
      case 1:  //单击
        rgb_flag = 1;
        pixelRoll = 0;
        if (bleKeyboard.isConnected())
          bleKeyboard.sendDialReport(DIAL_PRESS);
        break;
      case 2:  //双击切换
        switch (lv_page) {
          case 0:  //首页
            lv_event_send(ui_Button1, LV_EVENT_CLICKED, 0);
            lv_page = 1;
            break;
          case 1:  //第二页
            lv_event_send(ui_Button2, LV_EVENT_CLICKED, 0);
            lv_page = 0;
            break;
          default:
            break;
        }
        break;
      case 3:  //长按
        rgb_flag = 2;
        break;
      case 4:  //松开
        rgb_flag = 0;
        rgb_off();
        Serial.println("DIAL_RELEASE");
        if (bleKeyboard.isConnected())
          bleKeyboard.sendDialReport(DIAL_RELEASE);
        break;
      default:
        break;
    }
    push_states = 0;
  }
  sensor.update();
  motor.loopFOC();
  float p;//最大力度限制
  p = -now_angle * 10;
  if(abs(p)>4)
    p = _sign(p) * 4;  
  motor.move(p);
  lv_timer_handler(); /* let the GUI do its work */
}
void display_init() {
  // Init Display
  gfx->begin();
  gfx->fillScreen(BLACK);

#ifdef TFT_BLK
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
#endif

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 10);
  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 10);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    ui_init();
    lv_img_set_pivot(ui_Image2, 16, 120); /*Rotate around the top left corner*/
    lv_meter_set_indicator_value(meter, line_indic, dial_angle);
    Serial.println("Setup done");
  }
}
void power_off()
{
  digitalWrite(OFF_PIN,HIGH);
  delay(50);
  digitalWrite(OFF_PIN,LOW);
  delay(150);
  digitalWrite(OFF_PIN,HIGH);
  delay(50);
  digitalWrite(OFF_PIN,LOW);
}