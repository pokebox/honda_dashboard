#ifndef UI_DASHBOARD_H
#define UI_DASHBOARD_H

#include "config.h"
#ifdef USE_DISPLAY
#include <lvgl.h>
class DashboardUI {
public:
    static DashboardUI& getInstance();
    
    void create();
    void update();
    void updateDoorStatus(uint8_t door_fl, uint8_t door_fr, uint8_t door_rl, uint8_t door_rr, uint8_t trunk);
    void setTitle(const char* title);
    
    // 获取UI元素指针（用于外部更新）
    lv_obj_t* getLabelRPM() const { return label_rpm; }
    lv_obj_t* getLabelSpeed() const { return label_speed; }
    lv_obj_t* getLabelGas() const { return label_gas; }
    lv_obj_t* getLabelBrake() const { return label_brake; }
    lv_obj_t* getLabelAccLat() const { return label_acc_lat; }
    lv_obj_t* getLabelAccLong() const { return label_acc_long; }
    lv_obj_t* getLabelGLat() const { return label_g_lat; }
    lv_obj_t* getLabelGLong() const { return label_g_long; }
    lv_obj_t* getLabelEngineTemp() const { return label_engine_temp; }
    lv_obj_t* getLabelIntakeTemp() const { return label_intake_temp; }
    lv_obj_t* getLabelFuelConsumed() const { return label_fuel_consumed; }
    lv_obj_t* getLabelSteerAng() const { return label_steer_ang; }
    lv_obj_t* getLabelTripDistance() const { return label_trip_distance; }
    
private:
    DashboardUI();
    ~DashboardUI() = default;
    
    // UI元素
    lv_obj_t* title_label;
    lv_obj_t* label_rpm;
    lv_obj_t* label_gas;
    lv_obj_t* label_brake;
    lv_obj_t* label_speed;
    lv_obj_t* label_acc_lat;
    lv_obj_t* label_acc_long;
    lv_obj_t* label_g_lat;
    lv_obj_t* label_g_long;
    lv_obj_t* label_doors[5];
    lv_obj_t* door_labels[5];
    lv_obj_t* label_engine_temp;
    lv_obj_t* label_intake_temp;
    lv_obj_t* label_fuel_consumed;
    lv_obj_t* label_steer_ang;
    lv_obj_t* label_trip_distance;
    
    // 私有方法
    lv_obj_t* createLabelGroup(lv_obj_t* parent, const char* title, const char* initial_text, 
                               lv_coord_t x, lv_coord_t y, lv_coord_t width = 120, lv_coord_t height = 60);
    void createDoorIndicators(lv_obj_t* parent, lv_coord_t start_x, lv_coord_t start_y);
    
    // 禁用复制和赋值
    DashboardUI(const DashboardUI&) = delete;
    DashboardUI& operator=(const DashboardUI&) = delete;
};
#endif // USE_DISPLAY
#endif // UI_DASHBOARD_H