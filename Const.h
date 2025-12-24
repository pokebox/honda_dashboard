#pragma once
#include <cstdint>

// CAN FRAME IDs
#define STEERING_SENSORS_ID 342
#define ENGINE_DATA_ID 344       // 注意：从 ENGINE_DATA_2_ID 改为 ENGINE_DATA_ID
#define WHEEL_SPEEDS_ID 464
#define VSA_STATUS_ID 420
#define POWERTRAIN_DATA_ID 380
#define VEHICLE_DYNAMICS_ID 490
#define GEARBOX_ID 401           // 注意：之前是 422，根据DBC应该是401
#define STANDSTILL_ID 432
#define DRIVE_MODES_ID 545
#define ENGINE_DATA_3_ID 804
#define CAR_SPEED_ID 777
#define DOORS_STATUS_ID 1029     // 添加车门状态ID

// 新增变速箱ID（根据DBC）
#define GEARBOX_422_ID 422       // 如果同时存在两个变速箱ID

typedef enum {
    GEAR_L = 32,
    GEAR_S = 16,
    GEAR_D = 8,
    GEAR_N = 4,
    GEAR_R = 2,
    GEAR_P = 1
} GEAR_SHIFTER_t;

typedef enum {
    ECON_OFF = 0, 
    ECON_ON = 3
} ECON_t;

// Enum for SCM_BUTTONS.CRUISE_BUTTONS
typedef enum {
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
typedef enum {
    CMBS_BUTTON_RELEASED = 0,
    CMBS_BUTTON_PRESSED = 3  
} CMBS_BUTTON_t;

// Enum for EPB_STATUS.EPB_STATE
typedef enum {
    EPB_STATE_OFF = 0,
    EPB_STATE_ENGAGING = 1,
    EPB_STATE_DISENGAGING = 2,
    EPB_STATE_ON = 3
} EPB_STATE_t;

// ID: 380 - POWERTRAIN_DATA
typedef struct {
    uint8_t PEDAL_GAS;              // 7|8@0+ (1,0) [0|255] EON
    uint16_t ENGINE_RPM;            // 23|16@0+ (1,0) [0|15000] "rpm" EON
    uint8_t GAS_PRESSED;            // 39|1@0+ (1,0) [0|1] EON
    uint8_t ACC_STATUS;             // 38|1@0+ (1,0) [0|1] EON (新增)
    uint8_t BOH_17C;                // 37|5@0+ (1,0) [0|1] EON (新增)
    uint8_t BRAKE_SWITCH;           // 32|1@0+ (1,0) [0|1] EON
    uint8_t BOH2_17C;               // 47|10@0+ (1,0) [0|1] EON (新增)
    uint8_t BRAKE_PRESSED;          // 53|1@0+ (1,0) [0|1] EON
    uint8_t BOH3_17C;               // 52|5@0+ (1,0) [0|1] EON (新增)
    uint8_t COUNTER;                // 61|2@0+ (1,0) [0|3] EON (新增)
    uint8_t CHECKSUM;               // 59|4@0+ (1,0) [0|15] EON (新增)
} POWERTRAIN_DATA;

// ID: 344 - ENGINE_DATA (原ENGINE_DATA_2)
typedef struct {
    float XMISSION_SPEED;           // 7|16@0+ (0.01,0) [0|250] "kph" EON
    uint16_t ENGINE_RPM;            // 23|16@0+ (1,0) [0|15000] "rpm" EON (新增)
    float XMISSION_SPEED2;          // 39|16@0+ (0.01,0) [0|250] "kph" EON (新增)
    uint16_t ODOMETER;              // 55|8@0+ (10,0) [0|2550] "m" (新增)
    uint8_t COUNTER;                // 61|2@0+ (1,0) [0|3] EON (新增)
    uint8_t CHECKSUM;               // 59|4@0+ (1,0) [0|15] EON (新增)
} ENGINE_DATA;

// ID: 420 - VSA_STATUS
typedef struct {
    uint16_t USER_BRAKE;            // 7|16@0+ (0.015625,-1.609375) [0|1000] EON
    uint8_t COMPUTER_BRAKING;       // 23|1@0+ (1,0) [0|1] EON (新增)
    uint8_t ESP_DISABLED;           // 28|1@0+ (1,0) [0|1] EON
    uint8_t BRAKE_HOLD_RELATED;     // 52|1@0+ (1,0) [0|1] XXX (新增)
    uint8_t BRAKE_HOLD_ACTIVE;      // 46|1@0+ (1,0) [0|1] EON (新增)
    uint8_t BRAKE_HOLD_ENABLED;     // 45|1@0+ (1,0) [0|1] EON (新增)
    uint8_t COUNTER;                // 61|2@0+ (1,0) [0|3] EON (新增)
    uint8_t CHECKSUM;               // 59|4@0+ (1,0) [0|15] EON (新增)
} VSA_STATUS;

// ID: 1029 - DOORS_STATUS
typedef struct {
    uint8_t DOOR_OPEN_FL;           // 37|1@0+ (1,0) [0|1] EON (新增)
    uint8_t DOOR_OPEN_FR;           // 38|1@0+ (1,0) [0|1] EON (新增)
    uint8_t DOOR_OPEN_RL;           // 39|1@0+ (1,0) [0|1] EON (新增)
    uint8_t DOOR_OPEN_RR;           // 40|1@0+ (1,0) [0|1] EON (新增)
    uint8_t TRUNK_OPEN;             // 41|1@0+ (1,0) [0|1] EON (新增)
    uint8_t COUNTER;                // 61|2@0+ (1,0) [0|3] EON (新增)
    uint8_t CHECKSUM;               // 59|4@0+ (1,0) [0|15] EON (新增)
} DOORS_STATUS;

// 其他现有结构体保持不变...
typedef struct {
    uint8_t GEAR_SHIFTER;           // 5|6@0+ (1,0) [0|63] "" EON
} GEARBOX;


typedef struct {
    uint8_t ENGINE_TEMP; 
    uint8_t INTAKE_TEMP; 
    float TRIP_FUEL_CONSUMED;
} ENGINE_DATA_3;



typedef struct {
    uint8_t WHEELS_MOVING;
} STANDSTILL;

typedef struct {
    uint8_t EPB_ACTIVE;   
    uint8_t EPB_STATE;
} EPB_STATUS;

typedef struct {
    float WHEEL_SPEED_FL;
    float WHEEL_SPEED_FR;
    float WHEEL_SPEED_RL;
    float WHEEL_SPEED_RR;
} WHEEL_SPEEDS;

typedef struct {
    float LAT_ACCEL;              
    float LONG_ACCEL;                          
} VEHICLE_DYNAMICS;

typedef struct {
    uint8_t ECON_ON;       
} DRIVE_MODES;

typedef struct {
    uint8_t WHEEL_SPEED_FL; 
    uint8_t WHEEL_SPEED_FR; 
    uint8_t WHEEL_SPEED_RL; 
    uint8_t WHEEL_SPEED_RR; 
} ROUGH_WHEEL_SPEED;

typedef struct {
    uint8_t CRUISE_BUTTONS;
    uint8_t CRUISE_SETTING;              
} SCM_BUTTONS;

typedef struct {
    uint8_t SEATBELT_DRIVER_LAMP;     
    uint8_t SEATBELT_PASS_UNLATCHED;  
    uint8_t SEATBELT_PASS_LATCHED;    
    uint8_t SEATBELT_DRIVER_UNLATCHED;
    uint8_t SEATBELT_DRIVER_LATCHED;  
    uint8_t PASS_AIRBAG_OFF;          
    uint8_t PASS_AIRBAG_ON;               
} SEATBELT_STATUS;

typedef struct {
    float ROUGH_CAR_SPEED_3;  
    //uint8_t LOCK_STATUS;        
    //uint8_t IMPERIAL_UNIT;    
} CAR_SPEED;

typedef struct {
    uint8_t DASHBOARD_ALERT;  
    uint8_t AUTO_HEADLIGHTS;  
    uint8_t HIGH_BEAM_HOLD;   
    uint8_t HIGH_BEAM_FLASH;  
    uint8_t HEADLIGHTS_ON;    
    uint8_t WIPER_SWITCH;      
} STALK_STATUS;

typedef struct {
    uint8_t WIPERS;       
    uint8_t LOW_BEAMS;    
    uint8_t HIGH_BEAMS;   
    uint8_t PARK_LIGHTS;   
} STALK_STATUS_2;


