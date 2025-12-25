#include "ui_dashboard.h"
#include "lv_conf.h"

DashboardUI& DashboardUI::getInstance() {
    static DashboardUI instance;
    return instance;
}

DashboardUI::DashboardUI() {
    // 初始化指针
    title_label = nullptr;
    label_rpm = nullptr;
    label_gas = nullptr;
    label_brake = nullptr;
    label_speed = nullptr;
    label_acc_lat = nullptr;
    label_acc_long = nullptr;
    label_g_lat = nullptr;
    label_g_long = nullptr;
    label_engine_temp = nullptr;
    label_intake_temp = nullptr;
    label_fuel_consumed = nullptr;
    label_steer_ang = nullptr;
    label_trip_distance = nullptr;
    
    for (int i = 0; i < 5; i++) {
        label_doors[i] = nullptr;
        door_labels[i] = nullptr;
    }
}

void DashboardUI::create() {
    lv_obj_t* scr = lv_scr_act();
    
    // 设置屏幕背景色
    lv_obj_set_style_bg_color(scr, lv_color_hex(0), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    
    // 创建标题
    title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "Honda 车辆状态监控");
    lv_obj_set_style_text_color(title_label, COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(title_label, &dengxian_14, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 5);
    
    // 创建UI组件
    label_rpm =   createLabelGroup(scr, "发动机转速", "0", 20, 30, 100, 60);
    label_speed = createLabelGroup(scr, "车速", "0 km/h", 130, 30, 100, 60);
    label_gas =   createLabelGroup(scr, "油门", "0 %",    240, 30, 100, 60);
    label_brake = createLabelGroup(scr, "刹车力度", "-",  360, 30, 100, 60);
    
    label_acc_lat =  createLabelGroup(scr, "横向加速度", "0.00 m/s²",  20, 100, 100, 50);
    label_acc_long = createLabelGroup(scr, "纵向加速度", "0.00 m/s²", 130, 100, 100, 50);
    label_g_lat =    createLabelGroup(scr, "横向G值", "0.00 g",       240, 100, 100, 50);
    label_g_long =   createLabelGroup(scr, "纵向G值", "0.00 g",       360, 100, 100, 50);
    
    createDoorIndicators(scr, 10, 160);
    
    label_engine_temp = createLabelGroup(scr, "水温", "--°C", 20, 220, 140, 45);
    label_intake_temp = createLabelGroup(scr, "进气温度", "--°C", 170, 220, 140, 45);
    label_fuel_consumed = createLabelGroup(scr, "油耗", "-- L", 320, 220, 140, 45);

    label_steer_ang = createLabelGroup(scr, "转向角度", "-- °", 20, 275, 220, 45);
    label_trip_distance = createLabelGroup(scr, "里程", "-- km", 250, 275, 210, 45);
}

lv_obj_t* DashboardUI::createLabelGroup(lv_obj_t* parent, const char* title, const char* initial_text, 
                                        lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height) {
    // 创建容器
    lv_obj_t* container = lv_obj_create(parent);
    lv_obj_set_size(container, width, height);
    lv_obj_set_pos(container, x, y);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 1, 0);
    lv_obj_set_style_border_color(container, COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_border_opa(container, LV_OPA_30, 0);
    lv_obj_set_style_radius(container, 6, 0);
    lv_obj_set_style_pad_all(container, 3, 0);
    
    // 创建标题标签
    lv_obj_t* title_label = lv_label_create(container);
    lv_label_set_text(title_label, title);
    lv_obj_set_style_text_color(title_label, COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(title_label, &dengxian_14, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 2);
    
    // 创建内容标签
    lv_obj_t* content_label = lv_label_create(container);
    lv_label_set_text(content_label, initial_text);
    lv_obj_set_style_text_color(content_label, COLOR_TEXT_MAIN, 0);
    lv_obj_set_style_text_font(content_label, &dengxian_14, 0);
    lv_obj_align(content_label, LV_ALIGN_CENTER, 0, 5);
    
    return content_label;
}

void DashboardUI::createDoorIndicators(lv_obj_t* parent, lv_coord_t start_x, lv_coord_t start_y) {
    const char* door_names[] = {"主驾", "副驾", "左后", "右后", "后备箱"};
    
    const int total_doors = 5;
    const lv_coord_t item_width = 85;
    const lv_coord_t item_height = 50;
    const lv_coord_t horizontal_spacing = 5;
    
    for (int i = 0; i < total_doors; i++) {
        lv_coord_t x = start_x + i * (item_width + horizontal_spacing);
        lv_coord_t y = start_y;
        
        lv_obj_t* door_container = lv_obj_create(parent);
        lv_obj_set_size(door_container, item_width, item_height);
        lv_obj_set_pos(door_container, x, y);
        lv_obj_set_style_bg_opa(door_container, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(door_container, 1, 0);
        lv_obj_set_style_border_color(door_container, COLOR_TEXT_SECONDARY, 0);
        lv_obj_set_style_border_opa(door_container, LV_OPA_30, 0);
        lv_obj_set_style_radius(door_container, 6, 0);
        
        lv_obj_t* indicator = lv_obj_create(door_container);
        lv_obj_set_size(indicator, 12, 12);
        lv_obj_set_style_bg_color(indicator, COLOR_DOOR_CLOSED, 0);
        lv_obj_set_style_bg_opa(indicator, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(indicator, 4, 0);
        lv_obj_align(indicator, LV_ALIGN_LEFT_MID, 6, 0);
        
        lv_obj_t* name_label = lv_label_create(door_container);
        lv_label_set_text(name_label, door_names[i]);
        lv_obj_set_style_text_color(name_label, COLOR_TEXT_MAIN, 0);
        lv_obj_set_style_text_font(name_label, &dengxian_14, 0);
        lv_obj_set_style_text_align(name_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(name_label, LV_ALIGN_TOP_RIGHT, 4, 0);
        
        door_labels[i] = indicator;
        label_doors[i] = name_label;
    }
}

void DashboardUI::updateDoorStatus(uint8_t door_fl, uint8_t door_fr, uint8_t door_rl, uint8_t door_rr, uint8_t trunk) {
    uint8_t door_states[] = {door_fl, door_fr, door_rl, door_rr, trunk};
    
    for (int i = 0; i < 5; i++) {
        if (door_labels[i]) {
            lv_color_t color = door_states[i] ? COLOR_DOOR_OPEN : COLOR_DOOR_CLOSED;
            lv_obj_set_style_bg_color(door_labels[i], color, 0);
        }
        if (label_doors[i]) {
            lv_color_t color = door_states[i] ? COLOR_DOOR_OPEN : COLOR_TEXT_MAIN;
            lv_obj_set_style_text_color(label_doors[i], color, 0);
        }
    }
}

void DashboardUI::setTitle(const char* title) {
    if (title_label) {
        lv_label_set_text(title_label, title);
    }
}

void DashboardUI::update() {
    // 基本的UI更新逻辑，具体数据更新在外部完成
}