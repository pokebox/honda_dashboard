#include "can_handler.h"

CanHandler& CanHandler::getInstance() {
    static CanHandler instance;
    return instance;
}

CanHandler::CanHandler() {}

bool CanHandler::init() {
    if (!CAN.begin()) {
        Serial.println("CAN总线初始化失败!");
        return false;
    }
    Serial.println("CAN总线初始化成功!");
    return true;
}

void CanHandler::run() {
    CAN.run();
}

String CanHandler::getVIN() {
    return CAN.obd2RequestVIN();
}