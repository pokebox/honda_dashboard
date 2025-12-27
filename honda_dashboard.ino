#include "config.h"
#include "display.h"
#include "ui_dashboard.h"
#include "honda_data.pb-c.h"
#include "carDataProcessor.hpp"
#include "esp_broadcast.hpp"


// 全局对象引用
DisplayManager& display = DisplayManager::getInstance();
DashboardUI& dashboard = DashboardUI::getInstance();
HondaCAN CAN;

CarDataProcessor carProcessor(CAN);

// CAN任务函数声明
void canTask(void *arg);

void setup() {
    Serial.begin(115200);
    Serial.println("启动本田车辆仪表...");
    
    // 初始化CAN总线
    if (!CAN.begin()) {
        Serial.println("CAN总线初始化失败!");
    }
    
    // 初始化按钮
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

    // 初始化显示
    if (!display.init()) {
        Serial.println("显示初始化失败!");
        ESP.restart();
    }
    
    // 创建CAN任务
    xTaskCreatePinnedToCore(
        canTask,
        "CAN_Task",
        4096,
        NULL,
        1,
        NULL,
        0
    );
    
    // 创建仪表盘界面
    dashboard.create();
    
    // 获取车辆VIN并更新标题
    String car_vin = CAN.obd2RequestVIN();
    Serial.printf("车辆VIN: %s\n", car_vin.c_str());
    String title = "Honda 车辆状态监控 - " + car_vin;
    dashboard.setTitle(title.c_str());
    
    Serial.println("系统启动完成!");
}

void loop() {
    static uint32_t last_tick = 0;
    uint32_t now = millis();
    
    // 更新LVGL tick
    if (now - last_tick >= 5) {
        lv_tick_inc(5);
        last_tick = now;
    }
    
    // 按钮处理
    if (digitalRead(BTN_PIN) == LOW) {
        display.toggleRotation();
        delay(200); // 防抖
    }
    
    // 更新仪表盘显示
    updateDashboard();
    
    // 处理LVGL任务
    lv_timer_handler();
    
    delay(1);
}


void canTask(void *arg) {
    Serial.println("CAN任务启动，运行在Core " + String(xPortGetCoreID()));
    uint32_t last_can_update = 0;
    while (1) {
        // 处理CAN数据
        CAN.run();

        if (CAN.updateTime == last_can_update) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        carProcessor.updateData();

        // 广播数据
#ifdef USE_BROADCAST
        // CarStatus *car_data = carProcessor.getCarStatus();
        // size_t packed_size = car_status__get_packed_size(car_data);
        ALLDATA *all_data = carProcessor.getAllData();
        size_t packed_size = all__data__get_packed_size(all_data);
        uint8_t *data = (uint8_t*)malloc(packed_size);
        if (data == NULL) {
            Serial.println("Failed to allocate memory for packet");
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        //car_status__pack(car_data, data);
        all__data__pack(all_data, data);

        if (!broadcast_peer.send_message(data, packed_size)) {
            Serial.println("Failed to broadcast message "+ String(packed_size));
        }
        // 释放缓冲区
        free(data);
#endif
        last_can_update = CAN.updateTime;
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void updateDashboard() {
    static uint32_t last_update = 0;
    uint32_t now = millis();
    
    // 限制更新频率
    if (now - last_update < 10) {
        return;
    }
    

    // 更新UI
    if (dashboard.getLabelRPM()) {
        char buf[16];
        sprintf(buf, "%d RPM", CAN.Powertrain.ENGINE_RPM);
        lv_label_set_text(dashboard.getLabelRPM(), buf);
    }
    
    if (dashboard.getLabelSpeed()) {
        char buf[16];
        sprintf(buf, "%.1f km/h", CAN.EngineData.XMISSION_SPEED);
        lv_label_set_text(dashboard.getLabelSpeed(), buf);
    }
    
    if (dashboard.getLabelGas()) {
        char buf[16];
        sprintf(buf, "%d %%", map(CAN.GasPedal2.CAR_GAS, 0, 255, 0, 100));
        lv_label_set_text(dashboard.getLabelGas(), buf);
    }
    
    if (dashboard.getLabelBrake()) {
        char buf[16];
        sprintf(buf, "%.1f", CAN.VsaStatus.USER_BRAKE);
        lv_label_set_text(dashboard.getLabelBrake(), buf);
    }
    
    if (dashboard.getLabelAccLat() && dashboard.getLabelAccLong()) {
        char lat_buf[16], long_buf[16], g_x[16], g_y[16];
        sprintf(lat_buf, "%.2f m/s²", CAN.VehicleDynamics.LAT_ACCEL);
        sprintf(long_buf, "%.2f m/s²", CAN.VehicleDynamics.LONG_ACCEL);
        sprintf(g_x, "%.2f g", CAN.VehicleDynamics.LAT_ACCEL / 9.81f);
        sprintf(g_y, "%.2f g", CAN.VehicleDynamics.LONG_ACCEL / 9.81f);
        
        lv_label_set_text(dashboard.getLabelAccLat(), lat_buf);
        lv_label_set_text(dashboard.getLabelAccLong(), long_buf);
        lv_label_set_text(dashboard.getLabelGLat(), g_x);
        lv_label_set_text(dashboard.getLabelGLong(), g_y);
    }
    
    // 更新传感器数据
    if (dashboard.getLabelEngineTemp()) {
        char buf[16];
        sprintf(buf, "%d°C", CAN.EngineDataThree.ENGINE_TEMP);
        lv_label_set_text(dashboard.getLabelEngineTemp(), buf);
    }
    
    if (dashboard.getLabelIntakeTemp()) {
        char buf[16];
        sprintf(buf, "%d°C", CAN.EngineDataThree.INTAKE_TEMP);
        lv_label_set_text(dashboard.getLabelIntakeTemp(), buf);
    }
    
    if (dashboard.getLabelFuelConsumed()) {
        char buf[16];
        sprintf(buf, "%.2f L", CAN.EngineDataThree.TRIP_FUEL_CONSUMED);
        lv_label_set_text(dashboard.getLabelFuelConsumed(), buf);
    }
    
    if (dashboard.getLabelSteerAng()) {
        char buf[16];
        sprintf(buf, "%.1f °", CAN.SteeringSensors.STEER_ANGLE);
        lv_label_set_text(dashboard.getLabelSteerAng(), buf);
    }
    
    if (dashboard.getLabelTripDistance()) {
        char buf[16];
        sprintf(buf, "%d km", CAN.Odometer.ODOMETER);
        lv_label_set_text(dashboard.getLabelTripDistance(), buf);
    }
    
    // 更新门状态
    dashboard.updateDoorStatus(
        CAN.DoorsStatus.DOOR_OPEN_FL,
        CAN.DoorsStatus.DOOR_OPEN_FR,
        CAN.DoorsStatus.DOOR_OPEN_RL,
        CAN.DoorsStatus.DOOR_OPEN_RR,
        CAN.DoorsStatus.TRUNK_OPEN
    );
    
    // 调试输出
    static uint32_t last_uart_update = 0;
    if (now - last_uart_update >= 1000) {
        Serial.printf("RPM: %d\n", CAN.Powertrain.ENGINE_RPM);
        last_uart_update = now;
    }
    
    last_update = now;
}