#pragma once
#include <cstdint>

// CAN FRAME IDs
// 快速周期0.04s左右频率更新，约为20Hz
// 344 380 | 304 330 148 274 320
#define ENGINE_DATA_ID 344 // 注意：从 ENGINE_DATA_2_ID 改为 ENGINE_DATA_ID
#define POWERTRAIN_DATA_ID 380
//增加的
#define GAS_PEDAL_2_ID 304
#define STEERING_SENSORS_ID 330
#define KINEMATICS_ALT_ID 148

// 0.08s左右频率更新，约为10Hz
// 420 432 464 490 | 427 493 450 | 470 476 507 428
#define VSA_STATUS_ID 420

#define STANDSTILL_ID 432
#define WHEEL_SPEEDS_ID 464
#define VEHICLE_DYNAMICS_ID 490

#define STEER_MOTOR_TORQUE_ID 427
#define HUD_SETTING_ID 493
#define EPB_STATUS_ID 450

// 0.16s左右频率更新，约为5Hz
// 545 | 597 662 | 57
#define DRIVE_MODES_ID 545

#define ROUGH_WHEEL_SPEED_ID 597
#define SCM_BUTTONS_ID 662

// 0.4s左右频率更新，约为2.5Hz
// 777 804 795 806 881 882 884 888 891 918 800 808 815 825 846 773
#define CAR_SPEED_ID 777
#define CRUISE_ID 804

#define SCM_FEEDBACK_ID 806
#define STALK_STATUS_ID 884
#define STALK_STATUS_2_ID 891
#define SEATBELT_STATUS_ID 773

// 0.8s左右频率更新，约为1.25Hz
// 929 983 985
// 1.2s左右频率更新，约为0.8Hz
// 1029 1115 1064 1024 1036 1108 1125 1127
#define DOORS_STATUS_ID 1029 // 添加车门状态ID
// 2s左右频率更新，约为0.5Hz
// 1302 1296 1365 1361 1633
#define ODOMETER_ID 1302

// 4s左右频率更新，约为0.25Hz
// 1424

// 变速箱ID
#define GEARBOX_ID 401     // 注意：之前是 422，根据DBC应该是401
#define GEARBOX_422_ID 422 // 如果同时存在两个变速箱ID

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

// Enum for SCM_BUTTONS.CRUISE_BUTTONS
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

// Enum for SCM_FEEDBACK.CBMS_BUTTON
typedef enum
{
    CMBS_BUTTON_RELEASED = 0,
    CMBS_BUTTON_PRESSED = 3
} CMBS_BUTTON_t;

// Enum for EPB_STATUS.EPB_STATE
typedef enum
{
    EPB_STATE_OFF = 0,
    EPB_STATE_ENGAGING = 1,
    EPB_STATE_DISENGAGING = 2,
    EPB_STATE_ON = 3
} EPB_STATE_t;

// ID: 380 - POWERTRAIN_DATA 动力总成数据
typedef struct
{
    uint8_t PEDAL_GAS;     // 油门踏板 [0|255]
    uint16_t ENGINE_RPM;   // 发动机转速 [0|15000] rpm
    uint8_t GAS_PRESSED;   // 
    uint8_t ACC_STATUS;    //
    uint8_t BOH_17C;       //
    uint8_t BRAKE_SWITCH;  //
    uint8_t BOH2_17C;      //
    uint8_t BRAKE_PRESSED; //
    uint8_t BOH3_17C;      //
    uint8_t COUNTER;       //
    uint8_t CHECKSUM;      //
} POWERTRAIN_DATA;

// ID: 344 - ENGINE_DATA 发动机数据
typedef struct
{
    float XMISSION_SPEED;  //
    uint16_t ENGINE_RPM;   //
    float XMISSION_SPEED2; //
    uint16_t ODOMETER;     //
    uint8_t COUNTER;       //
    uint8_t CHECKSUM;      //
} ENGINE_DATA;

// ID: 420 - VSA_STATUS 车辆状态数据
typedef struct
{
    uint16_t USER_BRAKE;        //
    uint8_t COMPUTER_BRAKING;   //
    uint8_t ESP_DISABLED;       //
    uint8_t BRAKE_HOLD_RELATED; //
    uint8_t BRAKE_HOLD_ACTIVE;  //
    uint8_t BRAKE_HOLD_ENABLED; //
    uint8_t COUNTER;            //
    uint8_t CHECKSUM;           //
} VSA_STATUS;

// ID: 1029 - DOORS_STATUS 车门状态数据
typedef struct
{
    uint8_t DOOR_OPEN_FL; //
    uint8_t DOOR_OPEN_FR; //
    uint8_t DOOR_OPEN_RL; //
    uint8_t DOOR_OPEN_RR; //
    uint8_t TRUNK_OPEN;   //
    uint8_t COUNTER;      //
    uint8_t CHECKSUM;     //
} DOORS_STATUS;

typedef struct
{
    uint8_t GEAR_SHIFTER; // 5|6@0+ (1,0) [0|63] "" EON
} GEARBOX;

typedef struct
{
    uint8_t ENGINE_TEMP;
    uint8_t INTAKE_TEMP;
    float TRIP_FUEL_CONSUMED;
} ENGINE_DATA_3;

typedef struct
{
    uint8_t WHEELS_MOVING;
} STANDSTILL;

typedef struct
{
    uint8_t EPB_ACTIVE;
    uint8_t EPB_STATE;
} EPB_STATUS;

typedef struct
{
    float WHEEL_SPEED_FL;    //kph
    float WHEEL_SPEED_FR;   //kph
    float WHEEL_SPEED_RL;   //kph
    float WHEEL_SPEED_RR;   //kph
} WHEEL_SPEEDS;     //车轮速度数据

typedef struct
{
    float LAT_ACCEL;    //m/s^2
    float LONG_ACCEL;   //m/s^2
} VEHICLE_DYNAMICS;     //车辆动力学数据

typedef struct
{
    uint8_t ECON_ON;
} DRIVE_MODES;

typedef struct
{
    uint8_t WHEEL_SPEED_FL;
    uint8_t WHEEL_SPEED_FR;
    uint8_t WHEEL_SPEED_RL;
    uint8_t WHEEL_SPEED_RR;
} ROUGH_WHEEL_SPEED;

typedef struct
{
    uint8_t CRUISE_BUTTONS;
    uint8_t CRUISE_SETTING;
} SCM_BUTTONS;

typedef struct
{
    uint8_t SEATBELT_DRIVER_LAMP;
    uint8_t SEATBELT_PASS_UNLATCHED;
    uint8_t SEATBELT_PASS_LATCHED;
    uint8_t SEATBELT_DRIVER_UNLATCHED;
    uint8_t SEATBELT_DRIVER_LATCHED;
    uint8_t PASS_AIRBAG_OFF;
    uint8_t PASS_AIRBAG_ON;
} SEATBELT_STATUS;

typedef struct
{
    float ROUGH_CAR_SPEED_3;
    // uint8_t LOCK_STATUS;
    // uint8_t IMPERIAL_UNIT;
} CAR_SPEED;

typedef struct
{
    uint8_t DASHBOARD_ALERT;
    uint8_t AUTO_HEADLIGHTS;
    uint8_t HIGH_BEAM_HOLD;
    uint8_t HIGH_BEAM_FLASH;
    uint8_t HEADLIGHTS_ON;
    uint8_t WIPER_SWITCH;
} STALK_STATUS;

typedef struct
{
    uint8_t WIPERS;
    uint8_t LOW_BEAMS;
    uint8_t HIGH_BEAMS;
    uint8_t PARK_LIGHTS;
} STALK_STATUS_2;
