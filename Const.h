#pragma once
#include <cstdint>

// CAN FRAME IDs
// 快速周期0.04s左右频率更新，约为20Hz
#define ENGINE_DATA_ID 344
#define ENGINE_DATA_2_ID 344
#define POWERTRAIN_DATA_ID 380
#define GAS_PEDAL_2_ID 304
#define STEERING_SENSORS_ID 330
#define KINEMATICS_ALT_ID 148

// 0.08s左右频率更新，约为10Hz
#define VSA_STATUS_ID 420
#define STANDSTILL_ID 432
#define WHEEL_SPEEDS_ID 464
#define VEHICLE_DYNAMICS_ID 490
#define STEER_MOTOR_TORQUE_ID 427
#define HUD_SETTING_ID 493
#define EPB_STATUS_ID 450

// 0.16s左右频率更新，约为5Hz
#define ECON_STATUS_ID 545 // DRIVE_MODES_ID
#define ROUGH_WHEEL_SPEED_ID 597
#define SCM_BUTTONS_ID 662

// 0.4s左右频率更新，约为2.5Hz
#define CAR_SPEED_ID 777
#define CRUISE_ID 804 // ENGINE_DATA_3_ID
#define SCM_FEEDBACK_ID 806
#define STALK_STATUS_ID 884
#define STALK_STATUS_2_ID 891
#define SEATBELT_STATUS_ID 773

// 0.8s左右频率更新，约为1.25Hz
#define DOORS_STATUS_ID 1029
#define ODOMETER_ID 1302

// 没有出现的
#define GEARBOX_ID 422
#define GEARBOX_CVT_ID 401

typedef enum
{
    GEAR_L = 32,
    GEAR_S = 16,
    GEAR_D = 8,
    GEAR_N = 4,
    GEAR_R = 2,
    GEAR_P = 1
} GEAR_SHIFTER_t;

typedef enum
{
    ECON_OFF = 0,
    ECON_ON = 3
} ECON_t;

typedef enum
{
    CRUISE_BUTTON_NONE = 0,
    CRUISE_BUTTON_MAIN = 1,
    CRUISE_BUTTON_CANCEL = 2,
    CRUISE_BUTTON_DECEL_SET = 3,
    CRUISE_BUTTON_ACCEL_RES = 4,
    CRUISE_BUTTON_TBD_5 = 5,
    CRUISE_BUTTON_TBD_6 = 6,
    CRUISE_BUTTON_TBD_7 = 7
} CRUISE_BUTTONS_t;

typedef enum
{
    CMBS_BUTTON_RELEASED = 0,
    CMBS_BUTTON_PRESSED = 3
} CMBS_BUTTON_t;

typedef enum
{
    EPB_STATE_OFF = 0,
    EPB_STATE_ENGAGING = 1,
    EPB_STATE_DISENGAGING = 2,
    EPB_STATE_ON = 3
} EPB_STATE_t;

// ================== 结构体定义 ==================

// ID: 380 - POWERTRAIN_DATA 动力总成数据 20Hz
typedef struct
{
    uint8_t PEDAL_GAS;     // 油门踏板 [0|255]
    uint16_t ENGINE_RPM;   // 发动机转速 [0|15000] rpm
    uint8_t GAS_PRESSED;   // 油门是否被按下 [0|1]
    uint8_t ACC_STATUS;    // ACC状态 [0|1]
    uint8_t BOH_17C;       // 位37-41 (5 bits)
    uint8_t BRAKE_SWITCH;  // 刹车开关 [0|1]
    uint16_t BOH2_17C;     // 位47-56 (10 bits)
    uint8_t BRAKE_PRESSED; // 刹车是否被按下 [0|1]
    uint8_t BOH3_17C;      // 位52-56 (5 bits)
} POWERTRAIN_DATA;

// ID: 344 - ENGINE_DATA 发动机数据 20Hz
typedef struct
{
    float XMISSION_SPEED;  // 变速箱速度 (kph)
    uint16_t ENGINE_RPM;   // 发动机转速 [0|15000] rpm
    float XMISSION_SPEED2; // 变速箱速度2 (kph)
    uint16_t ODOMETER;     // 里程表 (单位: 10米)
} ENGINE_DATA;

// ID: 304 - GAS_PEDAL_2 油门踏板数据2 20Hz
typedef struct
{
    int16_t ENGINE_TORQUE_ESTIMATE; // 发动机扭矩估计 [-1000|1000] Nm
    int16_t ENGINE_TORQUE_REQUEST;  // 发动机扭矩请求 [-1000|1000] Nm
    uint8_t CAR_GAS;                // 油门踏板 [0|255]
} GAS_PEDAL_2;

// ID: 330 - STEERING_SENSORS 转向传感器数据 20Hz
typedef struct
{
    float STEER_ANGLE;          // 转向角度 [-500|500] deg
    float STEER_ANGLE_RATE;     // 转向角速度 [-3000|3000] deg/s
    uint8_t STEER_SENSOR_STATUS_1; // 转向传感器状态1 [0|1]
    uint8_t STEER_SENSOR_STATUS_2; // 转向传感器状态2 [0|1]
    uint8_t STEER_SENSOR_STATUS_3; // 转向传感器状态3 [0|1]
    float STEER_WHEEL_ANGLE;    // 转向盘角度 [-500|500] deg
} STEERING_SENSORS;

// ID: 148 - KINEMATICS_ALT 运动学数据（替代） 20Hz
typedef struct
{
    float LAT_ACCEL;    // 横向加速度 [-20|20] m/s²
    float LONG_ACCEL;   // 纵向加速度 [-20|20] m/s²
} KINEMATICS_ALT;

// ID: 420 - VSA_STATUS 车辆稳定性控制系统状态 10Hz
typedef struct
{
    float USER_BRAKE;         // 用户刹车 [0|1000] (单位: 0.015625, 偏移: -1.609375)
    uint8_t COMPUTER_BRAKING; // 电脑刹车 [0|1]
    uint8_t ESP_DISABLED;     // ESP禁用 [0|1]
    uint8_t BRAKE_HOLD_RELATED; // 刹车保持相关 [0|1]
    uint8_t BRAKE_HOLD_ACTIVE;  // 刹车保持激活 [0|1]
    uint8_t BRAKE_HOLD_ENABLED; // 刹车保持启用 [0|1]
} VSA_STATUS;

// ID: 432 - STANDSTILL 静止状态 10Hz
typedef struct
{
    uint8_t CONTROLLED_STANDSTILL; // 受控静止 [0|1]
    uint8_t WHEELS_MOVING;         // 车轮移动 [0|1]
    uint8_t BRAKE_ERROR_1;         // 刹车错误1 [0|1]
    uint8_t BRAKE_ERROR_2;         // 刹车错误2 [0|1]
} STANDSTILL;

// ID: 464 - WHEEL_SPEEDS 车轮速度 10Hz
typedef struct
{
    float WHEEL_SPEED_FL; // 左前轮速度 [0|250] kph
    float WHEEL_SPEED_FR; // 右前轮速度 [0|250] kph
    float WHEEL_SPEED_RL; // 左后轮速度 [0|250] kph
    float WHEEL_SPEED_RR; // 右后轮速度 [0|250] kph
} WHEEL_SPEEDS;

// ID: 490 - VEHICLE_DYNAMICS 车辆动力学数据 10Hz
typedef struct
{
    float LAT_ACCEL;  // 横向加速度 [-20|20] m/s²
    float LONG_ACCEL; // 纵向加速度 [-20|20] m/s²
} VEHICLE_DYNAMICS;

// ID: 427 - STEER_MOTOR_TORQUE 转向电机扭矩 10Hz
typedef struct
{
    uint8_t CONFIG_VALID;    // 配置有效 [0|1]
    uint16_t MOTOR_TORQUE;   // 电机扭矩 [0|256]
    uint8_t OUTPUT_DISABLED; // 输出禁用 [0|1]
} STEER_MOTOR_TORQUE;

// ID: 450 - EPB_STATUS 电子驻车制动状态 10Hz
typedef struct
{
    uint8_t EPB_BRAKE_AND_PULL; // EPB刹车和拉动 [0|1]
    uint8_t EPB_ACTIVE;         // EPB激活 [0|1]
    uint8_t EPB_STATE;          // EPB状态 (枚举: 0=disengaged, 1=engaging, 2=disengaging, 3=engaged)
} EPB_STATUS;

// ID: 545 - ECON_STATUS 经济模式状态 5Hz
typedef struct
{
    uint8_t ECON_ON_2;  // 经济模式2 [0|3]
    uint8_t ECON_ON;    // 经济模式 [0|1]
} ECON_STATUS;

// ID: 597 - ROUGH_WHEEL_SPEED 粗略车轮速度 5Hz
typedef struct
{
    uint8_t WHEEL_SPEED_FL; // 左前轮速度 [0|255] kph
    uint8_t WHEEL_SPEED_FR; // 右前轮速度 [0|255] kph
    uint8_t WHEEL_SPEED_RL; // 左后轮速度 [0|255] kph
    uint8_t WHEEL_SPEED_RR; // 右后轮速度 [0|255] kph
    uint8_t SET_TO_X55;     // 设置为0x55 [0|255]
    uint8_t SET_TO_X55_2;   // 设置为0x55_2 [0|255]
    uint8_t LONG_COUNTER;   // 长计数器 [0|255]
} ROUGH_WHEEL_SPEED;

// ID: 662 - SCM_BUTTONS 方向盘控制按钮 5Hz
typedef struct
{
    uint8_t CRUISE_BUTTONS; // 巡航按钮 (枚举: 0=none, 1=main, 2=cancel, 3=decel_set, 4=accel_res)
    uint8_t CRUISE_SETTING; // 巡航设置 [0|3]
} SCM_BUTTONS;

// ID: 773 - SEATBELT_STATUS 安全带状态 2.5Hz
typedef struct
{
    bool SEATBELT_DRIVER_LAMP;      // 驾驶员安全带灯 [0|1]
    bool SEATBELT_PASS_UNLATCHED;   // 乘客安全带未锁 [0|1]
    bool SEATBELT_PASS_LATCHED;     // 乘客安全带已锁 [0|1]
    bool SEATBELT_DRIVER_UNLATCHED; // 驾驶员安全带未锁 [0|1]
    bool SEATBELT_DRIVER_LATCHED;   // 驾驶员安全带已锁 [0|1]
    bool PASS_AIRBAG_OFF;           // 乘客安全气囊关闭 [0|1]
    bool PASS_AIRBAG_ON;            // 乘客安全气囊开启 [0|1]
} SEATBELT_STATUS;

// ID: 777 - CAR_SPEED 车速 2.5Hz
typedef struct
{
    uint8_t ROUGH_CAR_SPEED;     // 粗略车速 [0|255] mph
    float CAR_SPEED;             // 车速 [0|65535] kph
    float ROUGH_CAR_SPEED_3;     // 粗略车速3 [0|65535] kph
    uint8_t ROUGH_CAR_SPEED_2;   // 粗略车速2 [0|255] mph
    uint8_t LOCK_STATUS;         // 锁定状态 [0|255]
    uint8_t IMPERIAL_UNIT;       // 英制单位 [0|1]
} CAR_SPEED;

// ID: 806 - SCM_FEEDBACK 方向盘控制反馈 2.5Hz
typedef struct
{
    bool DRIVERS_DOOR_OPEN; // 驾驶员门开 [0|1]
    bool REVERSE_LIGHT;     // 倒车灯 [0|1]
    uint8_t CMBS_BUTTON;       // CMBS按钮 (枚举: 0=released, 3=pressed)
    bool LEFT_BLINKER;      // 左转向灯 [0|1]
    bool RIGHT_BLINKER;     // 右转向灯 [0|1]
    bool MAIN_ON;           // 主开关开启 [0|1]
    bool PARKING_BRAKE_ON;  // 驻车制动开启 [0|1]

} SCM_FEEDBACK;

// ID: 884 - STALK_STATUS 拨杆状态 2.5Hz
typedef struct
{
    uint8_t DASHBOARD_ALERT;  // 仪表盘警报 [0|255]
    uint8_t AUTO_HEADLIGHTS;  // 自动头灯 [0|1]
    uint8_t HIGH_BEAM_HOLD;   // 远光保持 [0|1]
    uint8_t HIGH_BEAM_FLASH;  // 远光闪烁 [0|1]
    uint8_t HEADLIGHTS_ON;    // 头灯开启 [0|1]
    uint8_t WIPER_SWITCH;     // 雨刷开关 [0|3]

} STALK_STATUS;

// ID: 891 - STALK_STATUS_2 拨杆状态2 2.5Hz
typedef struct
{
    uint8_t WIPERS;       // 雨刷状态 (枚举: 0=Off, 2=Low, 4=High)
    bool LOW_BEAMS;    // 近光灯 [0|1]
    bool HIGH_BEAMS;   // 远光灯 [0|1]
    bool PARK_LIGHTS;  // 停车灯 [0|1]
} STALK_STATUS_2;

// ID: 1029 - DOORS_STATUS 车门状态 1Hz
typedef struct
{
    bool DOOR_OPEN_FL; // 左前门开 [0|1]
    bool DOOR_OPEN_FR; // 右前门开 [0|1]
    bool DOOR_OPEN_RL; // 左后门开 [0|1]
    bool DOOR_OPEN_RR; // 右后门开 [0|1]
    bool TRUNK_OPEN;   // 行李箱开 [0|1]
} DOORS_STATUS;

// ID: 1302 - ODOMETER 里程表 1Hz
typedef struct
{
    uint32_t ODOMETER; // 里程 [0|16777215] km
} ODOMETER;

// ID: 401 - GEARBOX_CVT CVT变速箱
typedef struct
{
    uint8_t SELECTED_P;           // P档选择 [0|1]
    uint8_t SELECTED_R;           // R档选择 [0|1]
    uint8_t SELECTED_N;           // N档选择 [0|1]
    uint8_t SELECTED_D;           // D档选择 [0|1]
    uint8_t FORWARD_DRIVING_MODE; // 前进驾驶模式 [0|1]
    uint8_t CVT_UNKNOWN_1;        // CVT未知1 [0|255]
    uint8_t CVT_UNKNOWN_2;        // CVT未知2 [0|255]
    uint8_t GEAR_SHIFTER;         // 档位选择器 (枚举: 1=P, 2=R, 3=N, 4=D, 7=L, 10=S)
    uint8_t SHIFTER_POSITION_VALID; // 换档位置有效 [0|1]
    uint8_t NOT_FORWARD_GEAR;     // 非前进档 [0|1]
    uint8_t CVT_UNKNOWN_3;        // CVT未知3 [0|3]
} GEARBOX_CVT;

// ID: 493 - HUD_SETTING HUD设置
typedef struct
{
    uint8_t IMPERIAL_UNIT; // 英制单位 [0|1]
} HUD_SETTING;

// ID: 804 - CRUISE 定速巡航数据
typedef struct
{
    uint8_t HUD_SPEED_KPH;        // HUD显示的车速（公里/小时）[0|255] "kph"
    uint8_t HUD_SPEED_MPH;        // HUD显示的车速（英里/小时）[0|255] "mph"
    uint16_t TRIP_FUEL_CONSUMED;  // 行程燃油消耗 [0|65535] 注意：实际是16位，范围[0|255]可能是注释错误
    uint8_t CRUISE_SPEED_PCM;     // PCM设置的巡航速度 [0|255] (255表示未设置速度)
    int8_t BOH2;                  // 未知数据2，有符号 [-128|127] "" 
    uint8_t BOH3;                 // 未知数据3 [0|255] ""
} CRUISE_DATA;

typedef struct
{
    uint8_t ENGINE_TEMP;
    uint8_t INTAKE_TEMP;
    float TRIP_FUEL_CONSUMED;
} ENGINE_DATA_3;

typedef struct
{
    uint8_t ECON_ON;
} DRIVE_MODES;

typedef struct {
    uint8_t GEAR_SHIFTER;           // 5|6@0+ (1,0) [0|63] "" EON
} GEARBOX;  
