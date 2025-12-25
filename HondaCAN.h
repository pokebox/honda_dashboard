#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "Const.h"
#include "PIDS.h"

class HondaCAN
{
public:
    HondaCAN();
    bool begin();
    void run();
    float obd2Request(uint8_t pid);
    String obd2RequestVIN();    // 请求VIN
    
    int LED_EN = 13;        // LED使能引脚
    int RX_PIN = 2;         // CAN接收引脚
    int TX_PIN = 1;         // CAN发送引脚
    int CAN_RS = 3;         // CAN速率选择引脚（LOW=高速模式，HIGH=低功耗监听模式）
    int SENSE_V_ANA = 35;   // 电压检测模拟输入引脚
    int CAN_BIT = 11;       // CAN帧标识符位数（11位或29位）

    // ================== 公开的数据结构 ==================
    
    // ID: 380 - 动力总成数据，包含油门、发动机转速、刹车状态等信息
    POWERTRAIN_DATA Powertrain;

    // ID: 777 - 车速数据，包含精确车速和粗略车速
    CAR_SPEED CarSpeed;
    
    // ID: 420 - 车辆稳定性控制系统状态，包含刹车、ESP、刹车保持等信息
    VSA_STATUS VsaStatus;
    
    // ID: （需在Const.h中定义） - 发动机数据3，包含发动机温度、进气温度、行程燃油消耗
    ENGINE_DATA_3 EngineDataThree;
    
    // ID: 344 - 发动机数据，包含变速箱速度、发动机转速、里程等信息
    ENGINE_DATA EngineData;
    
    // ID: 1029 - 车门状态，包含所有车门和行李箱的开闭状态
    DOORS_STATUS DoorsStatus;
    
    // ================== 新增的数据结构 ==================
    
    // ID: 304 - 油门踏板数据2，包含发动机扭矩估计、扭矩请求和油门踏板位置
    GAS_PEDAL_2 GasPedal2;
    
    // ID: 330 - 转向传感器数据，包含转向角度、角速度和传感器状态
    STEERING_SENSORS SteeringSensors;
    
    // ID: 148 - 运动学替代数据，包含横向和纵向加速度
    KINEMATICS_ALT KinematicsAlt;
    
    // ID: 432 - 静止状态，包含车轮移动状态和刹车错误状态
    STANDSTILL Standstill;
    
    // ID: 464 - 车轮速度数据，包含四个车轮的精确速度
    WHEEL_SPEEDS WheelSpeeds;
    
    // ID: 490 - 车辆动力学数据，包含横向和纵向加速度
    VEHICLE_DYNAMICS VehicleDynamics;
    
    // ID: 427 - 转向电机扭矩，包含电机扭矩值和状态
    STEER_MOTOR_TORQUE SteerMotorTorque;
    
    // ID: 450 - 电子驻车制动状态，包含EPB状态和活动状态
    EPB_STATUS EpbStatus;
    
    // ID: 545 - 经济模式状态，包含ECON模式开关状态
    ECON_STATUS EconStatus;
    DRIVE_MODES DriveModes;
    
    // ID: 597 - 粗略车轮速度，包含四个车轮的粗略速度
    ROUGH_WHEEL_SPEED RoughWheelSpeed;
    
    // ID: 662 - 方向盘控制按钮，包含巡航控制按钮状态
    SCM_BUTTONS ScmButtons;
    
    // ID: 773 - 安全带状态，包含安全带锁扣状态和安全气囊指示
    SEATBELT_STATUS SeatbeltStatus;
    
    // ID: 806 - 方向盘控制反馈，包含转向灯、倒车灯、CMBS按钮等状态
    SCM_FEEDBACK ScmFeedback;
    
    // ID: 884 - 拨杆状态，包含大灯、雨刷等开关状态
    STALK_STATUS StalkStatus;
    
    // ID: 891 - 拨杆状态2，包含雨刷、灯光等更多状态
    STALK_STATUS_2 StalkStatus2;
    
    // ID: 1302 - 里程表数据，包含车辆总里程
    ODOMETER Odometer;
    
    // ID: 401 - CVT变速箱详细状态，包含档位选择和CVT特定参数
    GEARBOX_CVT GearboxCvt;
    GEARBOX Gearbox;

    // ID: 493 - HUD设置，包含单位制设置（英制/公制）
    HUD_SETTING HudSetting;

    // ID: 804 - 定速巡航数据，包含HUD车速、巡航速度和燃油消耗
    CRUISE_DATA CruiseData;
private:
    // ================== 解析函数声明 ==================
    
    // 动力总成数据解析
    void parsePowertrainData(uint8_t data[8]);
    
    // 变速箱状态解析
    void parseGearbox(uint8_t data[8]);
    
    // 车速数据解析
    void parseCarSpeed(uint8_t data[8]);
    
    // 车辆稳定性控制系统状态解析
    void parseVsaStatus(uint8_t data[8]);
    
    // 驾驶模式状态解析
    void parseDriveModes(uint8_t data[8]);
    
    // 发动机数据3解析
    void parseEngineData3(uint8_t data[8]);
    
    // 发动机数据解析
    void parseEngineData(uint8_t data[8]);
    
    // 车门状态解析
    void parseDoorsStatus(uint8_t data[8]);
    
    // ================== 新增解析函数声明 ==================
    
    // 油门踏板数据2解析
    void parseGasPedal2(uint8_t data[8]);
    
    // 转向传感器数据解析
    void parseSteeringSensors(uint8_t data[8]);
    
    // 运动学替代数据解析
    void parseKinematicsAlt(uint8_t data[8]);
    
    // 静止状态解析
    void parseStandstill(uint8_t data[8]);
    
    // 车轮速度数据解析
    void parseWheelSpeeds(uint8_t data[8]);
    
    // 车辆动力学数据解析
    void parseVehicleDynamics(uint8_t data[8]);
    
    // 转向电机扭矩解析
    void parseSteerMotorTorque(uint8_t data[8]);
    
    // 电子驻车制动状态解析
    void parseEpbStatus(uint8_t data[8]);
    
    // 经济模式状态解析
    void parseEconStatus(uint8_t data[8]);
    
    // 粗略车轮速度解析
    void parseRoughWheelSpeed(uint8_t data[8]);
    
    // 方向盘控制按钮解析
    void parseScmButtons(uint8_t data[8]);
    
    // 安全带状态解析
    void parseSeatbeltStatus(uint8_t data[8]);
    
    // 方向盘控制反馈解析
    void parseScmFeedback(uint8_t data[8]);
    
    // 拨杆状态解析
    void parseStalkStatus(uint8_t data[8]);
    
    // 拨杆状态2解析
    void parseStalkStatus2(uint8_t data[8]);
    
    // 里程表数据解析
    void parseOdometer(uint8_t data[8]);
    
    // CVT变速箱详细状态解析
    void parseGearboxCvt(uint8_t data[8]);
    
    // HUD设置解析
    void parseHudSetting(uint8_t data[8]);
    void parseCruiseData(uint8_t data[8]);  // 解析定速巡航数据
    // ================== OBD2功能 ==================
    
    // 发送OBD2请求
    void obd2_send(uint8_t mode, uint8_t pid);
    
    // 接收OBD2响应
    bool obd2_receive();
    
    // OBD2响应消息缓冲区
    twai_message_t obdResponse;
};
