#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <HondaCAN.h>
#include <Arduino.h>

class CanHandler {
public:
    static CanHandler& getInstance();
    
    bool init();
    void run();

    String getVIN();
    HondaCAN CAN;
    
private:
    CanHandler();
    ~CanHandler() = default;
    
    
    // 禁用复制和赋值
    CanHandler(const CanHandler&) = delete;
    CanHandler& operator=(const CanHandler&) = delete;
};

#endif // CAN_HANDLER_H