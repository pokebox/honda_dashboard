#include "config.h"
#include "display.h"
#include "can_handler.h"
#include "ui_dashboard.h"

// 全局对象引用
DisplayManager& display = DisplayManager::getInstance();
CanHandler& canHandler = CanHandler::getInstance();
DashboardUI& dashboard = DashboardUI::getInstance();

// CAN任务函数声明
void canTask(void *arg);

#ifdef USE_BROADCAST
#include "ESP32_NOW.h"
#include "WiFi.h"

#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

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
    if (!ESP_NOW.begin((const uint8_t *)ESPNOW_PMK) || !add()) {
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


void setup() {
    Serial.begin(115200);
    Serial.println("启动本田车辆仪表...");
    
    // 初始化CAN总线
    if (!canHandler.init()) {
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
    String car_vin = canHandler.getVIN();
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
    
    while (1) {
        // 处理CAN数据
        canHandler.run();
        
        // 广播数据
#ifdef USE_BROADCAST
        char data[16];
        snprintf(data, sizeof(data), "%d rpm", canHandler.getEngineRPM());
        if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
            Serial.println("Failed to broadcast message");
        }
#endif
        
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
    
    // 获取车辆数据
    uint16_t rpm = canHandler.getEngineRPM();
    float speed = canHandler.getSpeed();
    float lat_acc = canHandler.getAccelerationLat();
    float long_acc = canHandler.getAccelerationLong();
    uint8_t engine_temp = canHandler.getEngineTemp();
    uint8_t intake_temp = canHandler.getIntakeTemp();
    float fuel_consumed = canHandler.getFuelConsumed();
    float steering_angle = canHandler.getSteeringAngle();
    uint32_t trip_distance = canHandler.getTripDistance();
    float user_brake = canHandler.getUserBrake();
    uint8_t gas_pedal = canHandler.getGasPedal();
    
    // 更新UI
    if (dashboard.getLabelRPM()) {
        char buf[16];
        sprintf(buf, "%d RPM", rpm);
        lv_label_set_text(dashboard.getLabelRPM(), buf);
    }
    
    if (dashboard.getLabelSpeed()) {
        char buf[16];
        sprintf(buf, "%.1f km/h", speed);
        lv_label_set_text(dashboard.getLabelSpeed(), buf);
    }
    
    if (dashboard.getLabelGas()) {
        char buf[16];
        sprintf(buf, "%d %%", map(gas_pedal, 0, 255, 0, 100));
        lv_label_set_text(dashboard.getLabelGas(), buf);
    }
    
    if (dashboard.getLabelBrake()) {
        char buf[16];
        sprintf(buf, "%.1f", user_brake);
        lv_label_set_text(dashboard.getLabelBrake(), buf);
    }
    
    if (dashboard.getLabelAccLat() && dashboard.getLabelAccLong()) {
        char lat_buf[16], long_buf[16], g_x[16], g_y[16];
        sprintf(lat_buf, "%.2f m/s²", lat_acc);
        sprintf(long_buf, "%.2f m/s²", long_acc);
        sprintf(g_x, "%.2f g", lat_acc / 9.81f);
        sprintf(g_y, "%.2f g", long_acc / 9.81f);
        
        lv_label_set_text(dashboard.getLabelAccLat(), lat_buf);
        lv_label_set_text(dashboard.getLabelAccLong(), long_buf);
        lv_label_set_text(dashboard.getLabelGLat(), g_x);
        lv_label_set_text(dashboard.getLabelGLong(), g_y);
    }
    
    // 更新传感器数据
    if (dashboard.getLabelEngineTemp()) {
        char buf[16];
        sprintf(buf, "%d°C", engine_temp);
        lv_label_set_text(dashboard.getLabelEngineTemp(), buf);
    }
    
    if (dashboard.getLabelIntakeTemp()) {
        char buf[16];
        sprintf(buf, "%d°C", intake_temp);
        lv_label_set_text(dashboard.getLabelIntakeTemp(), buf);
    }
    
    if (dashboard.getLabelFuelConsumed()) {
        char buf[16];
        sprintf(buf, "%.1f L", fuel_consumed);
        lv_label_set_text(dashboard.getLabelFuelConsumed(), buf);
    }
    
    if (dashboard.getLabelSteerAng()) {
        char buf[16];
        sprintf(buf, "%.1f °", steering_angle);
        lv_label_set_text(dashboard.getLabelSteerAng(), buf);
    }
    
    if (dashboard.getLabelTripDistance()) {
        char buf[16];
        sprintf(buf, "%d km", trip_distance);
        lv_label_set_text(dashboard.getLabelTripDistance(), buf);
    }
    
    // 更新门状态
    dashboard.updateDoorStatus(
        canHandler.getDoorFL(),
        canHandler.getDoorFR(),
        canHandler.getDoorRL(),
        canHandler.getDoorRR(),
        canHandler.getTrunk()
    );
    
    // 调试输出
    static uint32_t last_uart_update = 0;
    if (now - last_uart_update >= 100) {
        Serial.printf(
            "刹车: %.1f, 电脑刹车: %d, ESP禁用: %d, 刹车保持相关: %d, 刹车保持激活: %d, 刹车保持启用: %d\n",
            canHandler.getUserBrake(),
            // 注意：这里需要添加相应的CAN数据访问方法
            // CAN.VsaStatus.COMPUTER_BRAKING,
            // CAN.VsaStatus.ESP_DISABLED,
            // CAN.VsaStatus.BRAKE_HOLD_RELATED,
            // CAN.VsaStatus.BRAKE_HOLD_ACTIVE,
            // CAN.VsaStatus.BRAKE_HOLD_ENABLED
            0, 0, 0, 0, 0  // 临时占位符
        );
        last_uart_update = now;
    }
    
    last_update = now;
}