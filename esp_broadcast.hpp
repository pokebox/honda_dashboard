#include "config.h"

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
#include <BLE2902.h>

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
    BLEDevice::startAdvertising();
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
  BLE_CAN_Characteristic = BLE_service->createCharacteristic(/*BLEUUID((uint16_t)0x1)*/(uint16_t)0x1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY /* | BLECharacteristic::PROPERTY_INDICATE*/);
  BLE_CAN_Characteristic->addDescriptor(new BLE2902());

  BLE_service->start();

  BLEAdvertising *BLE_advertising = BLEDevice::getAdvertising();
  BLE_advertising->addServiceUUID(RACECHRONO_UUID);
  BLE_advertising->setScanResponse(false);
  BLE_advertising->setMinInterval(100);
  BLE_advertising->setMaxInterval(100);
  BLEDevice::startAdvertising();
}
#endif
