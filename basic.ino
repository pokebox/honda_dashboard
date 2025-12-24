#include <HondaCAN.h>
#include <lvgl.h>
#include "lv_conf.h"
#include "SPI.h"

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


// 颜色定义
#define COLOR_BACKGROUND lv_color_hex(0x1a1a2e)    // 深蓝背景
#define COLOR_TEXT_MAIN lv_color_hex(0xffffff)     // 白色主文字
#define COLOR_TEXT_SECONDARY lv_color_hex(0xcccccc) // 灰色次要文字
#define COLOR_ACCENT lv_color_hex(0x00ff88)        // 绿色强调色
#define COLOR_DOOR_OPEN lv_color_hex(0xff5555)     // 红色（门开）
#define COLOR_DOOR_CLOSED lv_color_hex(0x55ff55)   // 绿色（门关）

// 全局标签指针
lv_obj_t* label_rpm;
lv_obj_t* label_gear;
lv_obj_t* label_speed;
lv_obj_t* label_acc_lat;
lv_obj_t* label_acc_long;

// 门状态标签
lv_obj_t* label_doors[5];  // 0:驾驶门, 1:副驾门, 2:左后门, 3:右后门, 4:后备箱
lv_obj_t* door_labels[5];  // 对应的门标签

// 传感器数据标签
lv_obj_t* label_engine_temp;
lv_obj_t* label_intake_temp;
lv_obj_t* label_fuel_consumed;
lv_obj_t* label_transmission_speed;
lv_obj_t* label_trip_distance;

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

// 创建带标题的标签组
lv_obj_t* create_label_group(lv_obj_t* parent, const char* title, const char* initial_text, 
                             lv_coord_t x, lv_coord_t y, lv_coord_t width = 120, lv_coord_t height = 60) {
    // 创建容器
    lv_obj_t* container = lv_obj_create(parent);
    lv_obj_set_size(container, width, height);
    lv_obj_set_pos(container, x, y);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 1, 0);
    lv_obj_set_style_border_color(container, COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_border_opa(container, LV_OPA_30, 0);
    lv_obj_set_style_radius(container, 6, 0);
    lv_obj_set_style_pad_all(container, 3, 0);  // 减少内边距
    
    // 创建标题标签
    lv_obj_t* title_label = lv_label_create(container);
    lv_label_set_text(title_label, title);
    lv_obj_set_style_text_color(title_label, COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(title_label, &dengxian_14, 0);  // 使用更小的字体
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 2);  // 减少顶部边距
    
    // 创建内容标签
    lv_obj_t* content_label = lv_label_create(container);
    lv_label_set_text(content_label, initial_text);
    lv_obj_set_style_text_color(content_label, COLOR_TEXT_MAIN, 0);
    lv_obj_set_style_text_font(content_label, &dengxian_14, 0);
    lv_obj_align(content_label, LV_ALIGN_CENTER, 0, 5);
    
    return content_label;
}

// 修改后的创建门状态指示器函数 - 5个门全部放在一行
void create_door_indicators(lv_obj_t* parent, lv_coord_t start_x, lv_coord_t start_y) {
    const char* door_names[] = {"主驾", "副驾", "左后", "右后", "后备箱"};
    
    // 计算门状态指示器的布局 - 5个门全部放在一行
    const int total_doors = 5;
    const lv_coord_t screen_width = 480;
    const lv_coord_t item_width = 85;   // 每个门状态容器的宽度
    const lv_coord_t item_height = 50;  // 每个门状态容器的高度
    
    // 计算每个门之间的间距
    // 总占用宽度 = 5个门 * 80 = 400像素
    // 剩余空间 = 480 - 400 = 80像素
    // 需要4个间距，每个间距 = 80 / 4 = 20像素
    const lv_coord_t horizontal_spacing = 5;
    
    for (int i = 0; i < total_doors; i++) {
        // 计算位置：起始x + i*(宽度+间距)
        lv_coord_t x = start_x + i * (item_width + horizontal_spacing);
        lv_coord_t y = start_y;
        
        // 创建门状态容器
        lv_obj_t* door_container = lv_obj_create(parent);
        lv_obj_set_size(door_container, item_width, item_height);
        lv_obj_set_pos(door_container, x, y);
        lv_obj_set_style_bg_opa(door_container, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(door_container, 1, 0);
        lv_obj_set_style_border_color(door_container, COLOR_TEXT_SECONDARY, 0);
        lv_obj_set_style_border_opa(door_container, LV_OPA_30, 0);
        lv_obj_set_style_radius(door_container, 6, 0);
        
        // 创建状态指示灯
        lv_obj_t* indicator = lv_obj_create(door_container);
        lv_obj_set_size(indicator, 12, 12);
        lv_obj_set_style_bg_color(indicator, COLOR_DOOR_CLOSED, 0);
        lv_obj_set_style_bg_opa(indicator, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(indicator, 4, 0);
        lv_obj_align(indicator, LV_ALIGN_LEFT_MID, 6, 0);
        
        // 创建门名称标签
        lv_obj_t* name_label = lv_label_create(door_container);
        lv_label_set_text(name_label, door_names[i]);
        lv_obj_set_style_text_color(name_label, COLOR_TEXT_MAIN, 0);
        lv_obj_set_style_text_font(name_label, &dengxian_14, 0);
        lv_obj_set_style_text_align(name_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(name_label, LV_ALIGN_TOP_RIGHT /*LV_ALIGN_RIGHT_MID*/, 4, 0);
        
        door_labels[i] = indicator;  // 保存指示灯对象
        label_doors[i] = name_label;  // 保存标签对象
    }
}

// 更新门状态颜色
void update_door_status(uint8_t door_fl, uint8_t door_fr, uint8_t door_rl, uint8_t door_rr, uint8_t trunk) {
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

void create_dashboard() {
    lv_obj_t* scr = lv_scr_act();
    
    // 设置屏幕背景色
    lv_obj_set_style_bg_color(scr, COLOR_BACKGROUND, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    
    // 创建标题 - 顶部居中
    lv_obj_t* title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "Honda 车辆状态监控");
    lv_obj_set_style_text_color(title_label, COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(title_label, &dengxian_14, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 5);
    
    // 第一行：主要驾驶信息 - 使用网格布局
    // 3列：转速 | 车速 | 档位
    label_rpm = create_label_group(scr, "发动机转速", "0", 20, 30, 140, 60);
    label_speed = create_label_group(scr, "车速", "0 km/h", 170, 30, 140, 60);
    label_gear = create_label_group(scr, "当前档位", "-", 320, 30, 140, 60);
    
    // 第二行：加速度信息 - 使用2列布局
    label_acc_lat = create_label_group(scr, "横向加速度", "0.00 m/s²", 20, 100, 220, 50);
    label_acc_long = create_label_group(scr, "纵向加速度", "0.00 m/s²", 250, 100, 210, 50);

    // 第三行：5个门状态全部放在同一行
    // 更新create_door_indicators函数调用，传递起始位置
    create_door_indicators(scr, 10, 160);
    
    // 第四行：传感器数据 - 使用3列布局
    label_engine_temp = create_label_group(scr, "水温", "--°C", 20, 220, 140, 45);
    label_intake_temp = create_label_group(scr, "进气温度", "--°C", 170, 220, 140, 45);
    label_fuel_consumed = create_label_group(scr, "油耗", "-- L", 320, 220, 140, 45);

    // 第五行：其他信息 - 使用2列布局
    label_transmission_speed = create_label_group(scr, "变速箱速度", "-- km/h", 20, 275, 220, 45);
    label_trip_distance = create_label_group(scr, "本次里程", "-- km", 250, 275, 210, 45);

}

// 档位值转字符
char gear_to_char(uint8_t gear) {
    switch(gear) {
        case 1: return 'P';
        case 2: return 'R';
        case 4: return 'N';
        case 8: return 'D';
        case 16: return 'S';
        case 32: return 'L';
        default: return '?';
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("启动本田车辆仪表...");
    
    // 初始化CAN总线
    if(!CAN.begin()) {
        Serial.println("CAN总线初始化失败!");
    } else {
        Serial.println("CAN总线初始化成功!");
    }

    // TFT背光
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, HIGH);

    // 初始化显示屏
    tft->begin();
    tft->fillScreen(BLACK);
    tft->setRotation(1);
    Serial.println("显示屏初始化完成");

    // 初始化LVGL
    lv_init();
    Serial.println("LVGL初始化完成");
    
    // 初始化显示缓冲区
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);
    Serial.println("显示缓冲区初始化完成");

    // 注册显示驱动
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    Serial.println("显示驱动注册完成");

    // 创建仪表盘界面
    create_dashboard();
    Serial.println("系统启动完成!");
}

void update_dashboard() {
    static uint32_t last_update = 0;
    uint32_t now = millis();
    
    // 调整更新频率
    // if (now - last_update < 20) {
    //     return;
    // }
  
    // 获取车辆数据
    uint16_t rpm = CAN.Powertrain.ENGINE_RPM;
    float speed = CAN.CarSpeed.ROUGH_CAR_SPEED_3;
    uint8_t gear = CAN.Gearbox.GEAR_SHIFTER;
    
    // 获取传感器数据
    uint8_t engine_temp = CAN.EngineDataThree.ENGINE_TEMP;
    uint8_t intake_temp = CAN.EngineDataThree.INTAKE_TEMP;
    float fuel_consumed = CAN.EngineDataThree.TRIP_FUEL_CONSUMED;
    float transmission_speed = CAN.EngineData.XMISSION_SPEED;
    uint8_t trip_distance = CAN.EngineData.ODOMETER;    // 里程m
    bool door_fl = CAN.DoorsStatus.DOOR_OPEN_FL;
    bool door_fr = CAN.DoorsStatus.DOOR_OPEN_FR;
    bool door_rl = CAN.DoorsStatus.DOOR_OPEN_RL;
    bool door_rr = CAN.DoorsStatus.DOOR_OPEN_RR;
    bool trunk  =  CAN.DoorsStatus.TRUNK_OPEN;
    
    // 更新转速显示
    if(label_rpm) {
        char buf[16];
        sprintf(buf, "%d RPM", rpm);
        lv_label_set_text(label_rpm, buf);
    }
    
    // 更新车速显示
    if(label_speed) {
        char buf[16];
        sprintf(buf, "%.0f km/h", speed);
        lv_label_set_text(label_speed, buf);
    }
    
    // 更新档位显示
    if(label_gear) {
      #ifdef IS_AUTO_GEAR_MODE
        char gear_char = gear_to_char(gear);
        char gear_text[4] = {gear_char, '\0'};
        
        // 根据档位设置颜色
        lv_color_t gear_color = COLOR_TEXT_MAIN;
        switch(gear_char) {
            case 'R': gear_color = lv_color_hex(0xff5555); break; // 红色
            case 'N': gear_color = lv_color_hex(0xffff55); break; // 黄色
            case 'D': gear_color = COLOR_ACCENT; break;          // 绿色
            case 'S': gear_color = lv_color_hex(0xffaa00); break; // 橙色
            case 'L': gear_color = lv_color_hex(0x5555ff); break; // 蓝色
        }
      #else
        char gear_text[2];
        sprintf(gear_text, "%d", gear);
        lv_color_t gear_color = COLOR_TEXT_MAIN;
      #endif

        
        lv_label_set_text(label_gear, gear_text);
        lv_obj_set_style_text_color(label_gear, gear_color, 0);
    }
    
    // 更新加速度显示（这里需要从CAN获取实际数据）
    if(label_acc_lat && label_acc_long) {
        // 暂时使用示例数据
        static float lat_acc = 0.0f;
        static float long_acc = 0.1f;
        
        char lat_buf[16], long_buf[16];
        sprintf(lat_buf, "%.2f m/s²", lat_acc);
        sprintf(long_buf, "%.2f m/s²", long_acc);
        
        lv_label_set_text(label_acc_lat, lat_buf);
        lv_label_set_text(label_acc_long, long_buf);
    }
    
    // 更新传感器数据显示
    if(label_engine_temp) {
        char buf[16];
        sprintf(buf, "%d°C", engine_temp);
        lv_label_set_text(label_engine_temp, buf);
    }
    
    if(label_intake_temp) {
        char buf[16];
        sprintf(buf, "%d°C", intake_temp);
        lv_label_set_text(label_intake_temp, buf);
    }
    
    if(label_fuel_consumed) {
        char buf[16];
        sprintf(buf, "%.1f L", fuel_consumed);
        lv_label_set_text(label_fuel_consumed, buf);
    }
    
    if(label_transmission_speed) {
        char buf[16];
        sprintf(buf, "%.1f km/h", transmission_speed);
        lv_label_set_text(label_transmission_speed, buf);
    }
    
    if(label_trip_distance) {
        char buf[16];
        sprintf(buf, "%d km", trip_distance);
        lv_label_set_text(label_trip_distance, buf);
    }
    
    update_door_status(door_fl, door_fr, door_rl, door_rr, trunk);

    
    // 串口输出调试信息（每秒一次）

        // Serial.printf("RPM: %d, Speed: %.1f, Gear: %d, Engine: %d°C, Intake: %d°C, Fuel: %.2fL\n",
        //   rpm, speed, gear, engine_temp, intake_temp, fuel_consumed);
    
    last_update = now;
}

void loop() {
    //static uint32_t last_tick = 0;
    //uint32_t now = millis();
    
    // 处理CAN消息
    CAN.run();
    // 更新LVGL tick（每5ms）
    // if (now - last_tick >= 5) {
        lv_tick_inc(1);
    //     last_tick = now;
    // }
    
    // 更新仪表盘显示
    update_dashboard();
    
    // 处理LVGL任务
    lv_timer_handler();
    
    delay(1);
}