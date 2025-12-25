#include <HondaCAN.h>
#include <lvgl.h>
#include "lv_conf.h"
#include "SPI.h"

#define USE_BROADCAST
//#define USE_BLE

#ifdef USE_BROADCAST
#include "ESP32_NOW.h"
#include "WiFi.h"

#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#define ESPNOW_WIFI_CHANNEL 6

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};
ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

#endif

#ifdef USE_BLE
#include <BLEServer.h>
#include <BLEDevice.h>

#define RACECHRONO_UUID "00001ff8-0000-1000-8000-00805f9b34fb"

bool deviceConnected = false;
bool oldDeviceConnected = false;

BLEServer *BLE_server = NULL;
BLECharacteristic *BLE_CAN_Characteristic = NULL; // RaceChrono CAN characteristic UUID 0x01

String device_name = "RC_DIY_" + String((uint16_t)((uint64_t)ESP.getEfuseMac() >> 32));

// Bluetooth Low Energy BLEServerCallbacks
class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *BLE_server)
  {
    deviceConnected = true;
    Serial.println("[I] Bluetooth client connected!");
  };

  void onDisconnect(BLEServer *BLE_server)
  {
    deviceConnected = false;
    Serial.println("[I] Bluetooth client disconnected!");
  }
};

// BLE configuration
void configBLE()
{
  BLEDevice::init(device_name.c_str());
  BLE_server = BLEDevice::createServer();
  BLE_server->setCallbacks(new ServerCallbacks());
  BLEService *BLE_service = BLE_server->createService(RACECHRONO_UUID);

  // GPS main characteristic definition
  BLE_CAN_Characteristic = BLE_service->createCharacteristic(BLEUUID((uint16_t)0x1), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  //BLE_CAN_Characteristic->addDescriptor(new BLE2902());

  BLE_service->start();

  BLEAdvertising *BLE_advertising = BLEDevice::getAdvertising();
  BLE_advertising->addServiceUUID(RACECHRONO_UUID);
  BLE_advertising->setScanResponse(false);
  BLE_advertising->setMinInterval(100);
  BLE_advertising->setMaxInterval(100);
  BLEDevice::startAdvertising();
}
#endif

#define TFT_DC 46
#define TFT_RST 21
#define TFT_CS 10
#define TFT_MOSI 11
#define TFT_MISO 9
#define TFT_CLK 12
#define TFT_LED 45
#define BTN_PIN 0

#define SPI_FREQUENCY 35000000

#include <Arduino_GFX_Library.h>
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_CLK, TFT_MOSI, TFT_MISO);
Arduino_GFX *tft = new Arduino_ILI9488_18bit(bus, TFT_RST, 1, false);

HondaCAN CAN;

static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];
static lv_disp_t * primary_disp = NULL; // 保存主显示对象

// 颜色定义
#define COLOR_BACKGROUND lv_color_hex(0x1a1a2e)    // 深蓝背景
#define COLOR_TEXT_MAIN lv_color_hex(0xffffff)     // 白色主文字
#define COLOR_TEXT_SECONDARY lv_color_hex(0xcccccc) // 灰色次要文字
#define COLOR_ACCENT lv_color_hex(0x00ff88)        // 绿色强调色
#define COLOR_DOOR_OPEN lv_color_hex(0xff5555)     // 红色（门开）
#define COLOR_DOOR_CLOSED lv_color_hex(0x55ff55)   // 绿色（门关）

// 全局标签指针
lv_obj_t* title_label;
lv_obj_t* label_rpm;
lv_obj_t* label_gas;
lv_obj_t* label_brake;
lv_obj_t* label_speed;
lv_obj_t* label_acc_lat;
lv_obj_t* label_acc_long;
lv_obj_t* label_g_lat;
lv_obj_t* label_g_long;

// 门状态标签
lv_obj_t* label_doors[5];  // 0:驾驶门, 1:副驾门, 2:左后门, 3:右后门, 4:后备箱
lv_obj_t* door_labels[5];  // 对应的门标签

// 传感器数据标签
lv_obj_t* label_engine_temp;
lv_obj_t* label_intake_temp;
lv_obj_t* label_fuel_consumed;
lv_obj_t* label_steer_ang;
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
    lv_obj_set_style_bg_color(scr, lv_color_hex(0), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    
    // 创建标题 - 顶部居中
    title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "Honda 车辆状态监控");
    lv_obj_set_style_text_color(title_label, COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(title_label, &dengxian_14, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 5);
    
    // 第一行：主要驾驶信息 - 使用网格布局
    // 3列：转速 | 车速 | 档位
    label_rpm =   create_label_group(scr, "发动机转速", "0", 20, 30, 100, 60);
    label_speed = create_label_group(scr, "车速", "0 km/h", 130, 30, 100, 60);
    label_gas =   create_label_group(scr, "油门", "0 %",    240, 30, 100, 60);
    label_brake = create_label_group(scr, "刹车力度", "-",  360, 30, 100, 60);
    
    // 第二行：加速度信息 - 使用2列布局
    label_acc_lat =  create_label_group(scr, "横向加速度", "0.00 m/s²",  20, 100, 100, 50);
    label_acc_long = create_label_group(scr, "纵向加速度", "0.00 m/s²", 130, 100, 100, 50);
    label_g_lat =    create_label_group(scr, "横向G值", "0.00 g",       240, 100, 100, 50);
    label_g_long =   create_label_group(scr, "纵向G值", "0.00 g",       360, 100, 100, 50);
    // 第三行：5个门状态全部放在同一行
    // 更新create_door_indicators函数调用，传递起始位置
    create_door_indicators(scr, 10, 160);
    
    // 第四行：传感器数据 - 使用3列布局
    label_engine_temp = create_label_group(scr, "水温", "--°C", 20, 220, 140, 45);
    label_intake_temp = create_label_group(scr, "进气温度", "--°C", 170, 220, 140, 45);
    label_fuel_consumed = create_label_group(scr, "油耗", "-- L", 320, 220, 140, 45);

    // 第五行：其他信息 - 使用2列布局
    label_steer_ang = create_label_group(scr, "转向角度", "-- °", 20, 275, 220, 45);
    label_trip_distance = create_label_group(scr, "里程", "-- km", 250, 275, 210, 45);

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
    pinMode(BTN_PIN, INPUT_PULLUP);

    #ifdef USE_BROADCAST
        WiFi.mode(WIFI_STA);
        WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
        while (!WiFi.STA.started()) {
            delay(100);
        }
        // Register the broadcast peer
        if (!broadcast_peer.begin()) {
            Serial.println("Failed to initialize broadcast peer");
            Serial.println("Reebooting in 5 seconds...");
            // delay(5000);
            ESP.restart();
        }
    #endif

    #ifdef USE_BLE
        configBLE();
    #endif
    // 初始化显示屏
    tft->begin(SPI_FREQUENCY);
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
    primary_disp = lv_disp_drv_register(&disp_drv);
    Serial.println("显示驱动注册完成");

    xTaskCreatePinnedToCore(
        canTask,          // 任务函数
        "CAN_Task",       // 任务名称
        4096,             // 堆栈大小
        NULL,             // 任务参数
        1,                // 优先级（1-24，数值越大优先级越高）
        NULL,             // 任务句柄
        0                 // 运行在Core 0
    );
    Serial.println("CAN任务创建到Core 0");
    // 创建仪表盘界面
    create_dashboard();
    Serial.println("系统启动完成!");
    String car_vin = CAN.obd2RequestVIN();
    Serial.printf("车辆VIN: %s\n", car_vin.c_str());
    String title = "Honda 车辆状态监控 - " + car_vin;
    lv_label_set_text(title_label, title.c_str());
}

void update_dashboard() {
    static uint32_t last_update = 0;
    uint32_t now = millis();
    
    // 调整更新频率
    if (now - last_update < 10) {
        return;
    }
  
    // 获取车辆数据
    uint16_t rpm = CAN.Powertrain.ENGINE_RPM;       //转速
    float speed = CAN.EngineData.XMISSION_SPEED;    //车速
    uint8_t gear = CAN.GearboxCvt.GEAR_SHIFTER;     //档位
    
    // 获取传感器数据
    uint8_t engine_temp = CAN.EngineDataThree.ENGINE_TEMP;      //水温
    uint8_t intake_temp = CAN.EngineDataThree.INTAKE_TEMP;      //进气温度
    float fuel_consumed = CAN.EngineDataThree.TRIP_FUEL_CONSUMED;   //油耗

    float transmission_speed = CAN.EngineData.XMISSION_SPEED2;      // 变速箱速度
    uint32_t trip_distance = CAN.Odometer.ODOMETER;    // 里程
    bool door_fl = CAN.DoorsStatus.DOOR_OPEN_FL;
    bool door_fr = CAN.DoorsStatus.DOOR_OPEN_FR;
    bool door_rl = CAN.DoorsStatus.DOOR_OPEN_RL;
    bool door_rr = CAN.DoorsStatus.DOOR_OPEN_RR;
    bool trunk  =  CAN.DoorsStatus.TRUNK_OPEN;

    float lat_acc = CAN.VehicleDynamics.LAT_ACCEL;
    float long_acc = CAN.VehicleDynamics.LONG_ACCEL;
    
    // 更新转速显示
    if(label_rpm) {
        char buf[16];
        sprintf(buf, "%d RPM", rpm);
        lv_label_set_text(label_rpm, buf);
    }
    
    // 更新车速显示
    if(label_speed) {
        char buf[16];
        sprintf(buf, "%.1f km/h", speed);
        lv_label_set_text(label_speed, buf);
    }
    
    if (label_gas) {
        char buf[16];
        sprintf(buf, "%d %%", map(CAN.GasPedal2.CAR_GAS, 0, 255, 0, 100));
        lv_label_set_text(label_gas, buf);
    }

    if(label_brake) {
        char brake_text[16];
        sprintf(brake_text, "%.1f", CAN.VsaStatus.USER_BRAKE);
        lv_label_set_text(label_brake, brake_text);
    }
    
    // 更新加速度显示
    if(label_acc_lat && label_acc_long) {
        
        char lat_buf[16], long_buf[16], g_x[16], g_y[16];
        sprintf(lat_buf, "%.2f m/s²", lat_acc);
        sprintf(long_buf, "%.2f m/s²", long_acc);
        sprintf(g_x, "%.2f g", lat_acc / 9.81f);
        sprintf(g_y, "%.2f g", long_acc / 9.81f);
        
        lv_label_set_text(label_acc_lat, lat_buf);
        lv_label_set_text(label_acc_long, long_buf);
        lv_label_set_text(label_g_lat, g_x);
        lv_label_set_text(label_g_long, g_y);
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
    
    if(label_steer_ang) {
        char buf[16];
        sprintf(buf, "%.1f °", CAN.SteeringSensors.STEER_ANGLE);
        lv_label_set_text(label_steer_ang, buf);
    }
    
    if(label_trip_distance) {
        char buf[16];
        sprintf(buf, "%d km", trip_distance);
        lv_label_set_text(label_trip_distance, buf);
    }
    
    update_door_status(door_fl, door_fr, door_rl, door_rr, trunk);

    
    // 串口输出调试信息（每秒一次）

    static uint32_t last_uart_update = 0;
    if (now - last_uart_update >= 100)
    {
        // Serial.printf(
        //     "转向角度: %.1f°, 方向盘角度: %.1f deg, 角速度: %.1f deg, 状态1: %d, %d, %d | 电机扭矩: %d, CFG: %d, EN: %d | 里程：%d\n",
        //     CAN.SteeringSensors.STEER_ANGLE,
        //     CAN.SteeringSensors.STEER_WHEEL_ANGLE,
        //     CAN.SteeringSensors.STEER_ANGLE_RATE,
        //     CAN.SteeringSensors.STEER_SENSOR_STATUS_1,
        //     CAN.SteeringSensors.STEER_SENSOR_STATUS_2,
        //     CAN.SteeringSensors.STEER_SENSOR_STATUS_3,

        //     CAN.SteerMotorTorque.MOTOR_TORQUE,
        //     CAN.SteerMotorTorque.CONFIG_VALID,
        //     CAN.SteerMotorTorque.OUTPUT_DISABLED,

        //     CAN.EngineData.ODOMETER
        //   );

        Serial.printf(
            "刹车: %.1f, 电脑刹车: %d, ESP禁用: %d, 刹车保持相关: %d, 刹车保持激活: %d, 刹车保持启用: %d\n",
            CAN.VsaStatus.USER_BRAKE,
            CAN.VsaStatus.COMPUTER_BRAKING,
            CAN.VsaStatus.ESP_DISABLED,
            CAN.VsaStatus.BRAKE_HOLD_RELATED,
            CAN.VsaStatus.BRAKE_HOLD_ACTIVE,
            CAN.VsaStatus.BRAKE_HOLD_ENABLED
        );
        last_uart_update = now;
    }
    last_update = now;
}

void canTask(void *arg)
{
    Serial.println("CAN任务启动，运行在Core " + String(xPortGetCoreID()));
    while (1)
    {
        CAN.run();
#ifdef USE_BROADCAST
        char data[16];
        snprintf(data, sizeof(data), "%d rpm", CAN.Powertrain.ENGINE_RPM);
        if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
            Serial.println("Failed to broadcast message");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
#endif
    }
}

void loop() {
    static uint32_t last_tick = 0;
    uint32_t now = millis();
    // 更新LVGL tick（每5ms）
    if (now - last_tick >= 5) {
        lv_tick_inc(5);
        last_tick = now;
    }
    if (digitalRead(BTN_PIN) == LOW)
    {
        // 按下按钮，切换屏幕方向
        if (tft->getRotation() == 1)
        {
            tft->setRotation(3);
        }
        else
        {
            tft->setRotation(1);
        }
        lv_obj_invalidate(lv_scr_act());
        lv_refr_now(primary_disp);
    }
    // 更新仪表盘显示
    update_dashboard();
    // 处理LVGL任务
    uint32_t start_time = millis();
    lv_timer_handler();
    Serial.printf("LVGL任务耗时：%dms\n", millis() - start_time);
    delay(1);
}