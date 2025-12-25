#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <HondaCAN.h>
#include <Arduino.h>

class CanHandler {
public:
    static CanHandler& getInstance();
    
    bool init();
    void run();
    
    // 获取数据的方法
    uint16_t getEngineRPM() const { return CAN.Powertrain.ENGINE_RPM; }
    float getSpeed() const { return CAN.EngineData.XMISSION_SPEED; }
    float getAccelerationLat() const { return CAN.VehicleDynamics.LAT_ACCEL; }
    float getAccelerationLong() const { return CAN.VehicleDynamics.LONG_ACCEL; }
    uint8_t getEngineTemp() const { return CAN.EngineDataThree.ENGINE_TEMP; }
    uint8_t getIntakeTemp() const { return CAN.EngineDataThree.INTAKE_TEMP; }
    float getFuelConsumed() const { return CAN.EngineDataThree.TRIP_FUEL_CONSUMED; }
    float getSteeringAngle() const { return CAN.SteeringSensors.STEER_ANGLE; }
    uint32_t getTripDistance() const { return CAN.Odometer.ODOMETER; }
    float getUserBrake() const { return CAN.VsaStatus.USER_BRAKE; }
    uint8_t getGasPedal() const { return CAN.GasPedal2.CAR_GAS; }
    
    // 门状态
    bool getDoorFL() const { return CAN.DoorsStatus.DOOR_OPEN_FL; }
    bool getDoorFR() const { return CAN.DoorsStatus.DOOR_OPEN_FR; }
    bool getDoorRL() const { return CAN.DoorsStatus.DOOR_OPEN_RL; }
    bool getDoorRR() const { return CAN.DoorsStatus.DOOR_OPEN_RR; }
    bool getTrunk() const { return CAN.DoorsStatus.TRUNK_OPEN; }
    
    String getVIN();
    
private:
    CanHandler();
    ~CanHandler() = default;
    
    HondaCAN CAN;
    
    // 禁用复制和赋值
    CanHandler(const CanHandler&) = delete;
    CanHandler& operator=(const CanHandler&) = delete;
};

#endif // CAN_HANDLER_H