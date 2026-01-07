#include "display.h"
#include <SPI.h>

#ifdef USE_DISPLAY
// 静态成员初始化
lv_disp_t* DisplayManager::primary_disp = nullptr;

// 显示缓冲区
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * SCREEN_HEIGHT / 10];

DisplayManager::DisplayManager() : bus(nullptr), tft(nullptr) {}

DisplayManager& DisplayManager::getInstance() {
    static DisplayManager instance;
    return instance;
}

bool DisplayManager::init() {
    // 初始化TFT
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, HIGH);
    
    bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_CLK, TFT_MOSI, TFT_MISO);
    tft = new Arduino_ILI9488_18bit(bus, TFT_RST, 1, false);
    
    tft->begin(SPI_FREQUENCY);
    tft->fillScreen(BLACK);
    tft->setRotation(1);
    
    // 初始化LVGL
    lv_init();
    
    // 初始化显示缓冲区
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * SCREEN_HEIGHT / 10);
    
    // 注册显示驱动
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = [](lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
        DisplayManager::getInstance().flushDisplay(disp_drv, area, color_p);
    };
    disp_drv.draw_buf = &draw_buf;
    primary_disp = lv_disp_drv_register(&disp_drv);
    
    return true;
}

void DisplayManager::flushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
    tft->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    tft->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

    lv_disp_flush_ready(disp_drv);
}

void DisplayManager::setRotation(uint8_t rotation) {
    if (tft) {
        tft->setRotation(rotation);
    }
}

uint8_t DisplayManager::getRotation() const {
    return tft ? tft->getRotation() : 0;
}

void DisplayManager::toggleRotation() {
    if (tft->getRotation() == 1) {
        tft->setRotation(3);
    } else {
        tft->setRotation(1);
    }
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(primary_disp);
}

lv_disp_t* DisplayManager::getPrimaryDisplay() const {
    return primary_disp;
}
#endif // USE_DISPLAY