#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include "config.h"

class DisplayManager {
public:
    static DisplayManager& getInstance();
    
    bool init();
    void setRotation(uint8_t rotation);
    uint8_t getRotation() const;
    void toggleRotation();
    void flushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
    lv_disp_t* getPrimaryDisplay() const;
    
private:
    DisplayManager();
    ~DisplayManager() = default;
    
    Arduino_DataBus *bus;
    Arduino_GFX *tft;
    static lv_disp_t *primary_disp;
    
    // 禁用复制和赋值
    DisplayManager(const DisplayManager&) = delete;
    DisplayManager& operator=(const DisplayManager&) = delete;
};

#endif // DISPLAY_H