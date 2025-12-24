#include <HondaCAN.h>
#include <lvgl.h>
#include "lv_conf.h"
#include "SPI.h"

// #define TFT_DC 46
// #define TFT_RST 21
// #define TFT_CS 10
// #define TFT_MOSI 11
// #define TFT_MISO 9
// #define TFT_CLK 12

#define TFT_DC 46
#define TFT_RST 21
#define TFT_CS 10
#define TFT_MOSI 11
#define TFT_MISO 9
#define TFT_CLK 12

#define TFT_LED 45

#include <Arduino_GFX_Library.h>
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_CLK, TFT_MOSI, TFT_MISO);
Arduino_GFX *tft = new Arduino_ILI9488_18bit(bus, TFT_RST, 1, false);

HondaCAN CAN;

static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

// 全局保存仪表盘控件指针
lv_obj_t* arc_rpm;
lv_obj_t* label_rpm;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
#ifndef DIRECT_RENDER_MODE
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  tft->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  tft->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
#endif // #ifndef DIRECT_RENDER_MODE

  lv_disp_flush_ready(disp_drv);
}

void create_dashboard()
{
    lv_obj_t * scr = lv_scr_act();
    
    // 创建圆弧作为转速表
    arc_rpm = lv_arc_create(scr);
    lv_arc_set_bg_angles(arc_rpm, 0, 270);
    lv_arc_set_angles(arc_rpm, 0, 0);
    lv_arc_set_range(arc_rpm, 0, 8000);
    lv_obj_set_size(arc_rpm, 200, 200);
    lv_obj_center(arc_rpm);
    
    // 设置圆弧样式
    lv_obj_set_style_arc_width(arc_rpm, 20, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc_rpm, 20, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc_rpm, lv_color_hex(0x505050), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc_rpm, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    
    // 创建转速数字标签
    label_rpm = lv_label_create(scr);
    lv_label_set_text(label_rpm, "RPM: 0");
    lv_obj_align(label_rpm, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_text_font(label_rpm, &lv_font_montserrat_14, 0);
}

void setup() {
    Serial.begin(115200);
    if(!CAN.begin()) Serial.println("Failed to install or start TWAI driver!");

    // TFT_LED 灯亮起
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, HIGH);
    tft->begin();
    tft->fillScreen(BLACK);
    tft->setRotation(1);

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    create_dashboard();
}

void loop() {
    CAN.run();

    uint16_t rpm = CAN.Powertrain.ENGINE_RPM;
    
    Serial.println(rpm);

    // 更新圆弧指示器
    if(arc_rpm) {
        // 将转速映射到角度 (0-8000 RPM 对应 0-270 度)
        int16_t angle = map(rpm, 0, 8000, 0, 270);
        lv_arc_set_value(arc_rpm, angle);
    }

    // 更新数字标签
    if(label_rpm) {
        char buf[20];
        sprintf(buf, "RPM: %d", rpm);
        lv_label_set_text(label_rpm, buf);
    }

    //lv_task_handler();
    lv_timer_handler();
    delay(5);
}