#include "HondaCAN.h"

HondaCAN::HondaCAN()
{
}

bool HondaCAN::begin(/*uint32_t filter = 0xFFFFFFFF*/)
{
  pinMode(CAN_RS, OUTPUT);
  digitalWrite(CAN_RS, LOW);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#if 1
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#else
  twai_filter_config_t f_config = {
    .acceptance_code = (0x000 << 21) | (0x000 << 5),
    .acceptance_mask = ~((0x400 << 21) | (0x7ff << 5)),
    .single_filter = false
  };
#endif
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    return false;
  if (twai_start() != ESP_OK)
    return false;
  return true;
}

void HondaCAN::run()
{
  twai_message_t message;
  bool updated = false;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    switch (message.identifier)
    {
    case POWERTRAIN_DATA_ID:
      this->parsePowertrainData(message.data);
      updated = true;
      break;
    case CAR_SPEED_ID:
      this->parseCarSpeed(message.data);
      updated = true;
      break;
    case VSA_STATUS_ID:
      this->parseVsaStatus(message.data);
      updated = true;
      break;
    case ENGINE_DATA_ID:
      this->parseEngineData(message.data);
      updated = true;
      break;
    case DOORS_STATUS_ID:
      this->parseDoorsStatus(message.data);
      updated = true;
      break;
    case GAS_PEDAL_2_ID:
      this->parseGasPedal2(message.data);
      updated = true;
      break;
    case STEERING_SENSORS_ID:
      this->parseSteeringSensors(message.data);
      updated = true;
      break;
    case KINEMATICS_ALT_ID:
      this->parseKinematicsAlt(message.data);
      updated = true;
      break;
    case STANDSTILL_ID:
      this->parseStandstill(message.data);
      updated = true;
      break;
    case WHEEL_SPEEDS_ID:
      this->parseWheelSpeeds(message.data);
      updated = true;
      break;
    case VEHICLE_DYNAMICS_ID:
      this->parseVehicleDynamics(message.data);
      updated = true;
      break;
    case STEER_MOTOR_TORQUE_ID:
      this->parseSteerMotorTorque(message.data);
      updated = true;
      break;
    case EPB_STATUS_ID:
      this->parseEpbStatus(message.data);
      updated = true;
      break;
    case ECON_STATUS_ID:
      this->parseEconStatus(message.data);
      this->parseDriveModes(message.data);
      updated = true;
      break;
    case ROUGH_WHEEL_SPEED_ID:
      this->parseRoughWheelSpeed(message.data);
      updated = true;
      break;
    case SCM_BUTTONS_ID:
      this->parseScmButtons(message.data);
      updated = true;
      break;
    case SEATBELT_STATUS_ID:
      this->parseSeatbeltStatus(message.data);
      updated = true;
      break;
    case SCM_FEEDBACK_ID:
      this->parseScmFeedback(message.data);
      updated = true;
      break;
    case STALK_STATUS_ID:
      this->parseStalkStatus(message.data);
      updated = true;
      break;
    case STALK_STATUS_2_ID:
      this->parseStalkStatus2(message.data);
      updated = true;
      break;
    case ODOMETER_ID:
      this->parseOdometer(message.data);
      updated = true;
      break;
    case GEARBOX_CVT_ID:
      this->parseGearboxCvt(message.data);
      updated = true;
      break;
    case GEARBOX_ID:
      this->parseGearbox(message.data);
      updated = true;
      break;
    case HUD_SETTING_ID:
      this->parseHudSetting(message.data);
      updated = true;
      break;
    case CRUISE_ID:
      this->parseCruiseData(message.data);
      this->parseEngineData3(message.data);
      updated = true;
      break;
    }
  }
  if (updated)
  {
    updateTime = millis();
    int gear = calculateHondaCivicGear(EngineData.ENGINE_RPM, EngineData.XMISSION_SPEED);
    if (gear <= 0) {
      gear = 0;
    }
    Gearbox.GEAR_SHIFTER = gear;
  }
}


/**
 * @brief 根据发动机转速和车速计算本田思域6MT的估计档位
 * @param rpm 发动机转速，单位：转/分钟 (RPM)
 * @param speed_kph 车速，单位：公里/小时 (km/h)
 * @return int 估计的档位 (1-6)。返回0表示车速或转速为0，返回-1表示计算出的传动比异常或不在正常行车范围。
 */
int calculateHondaCivicGear(int rpm, double speed_kph) {
    // -------- 核心参数配置 (需根据实车数据校准) --------
    // 主减速器最终传动比 (示例值，思域常见范围约为3.5-4.2)
    const double FINAL_DRIVE_RATIO = 4.105;
    // 轮胎滚动半径 (示例值，对应思域215/55 R16轮胎，单位：米)
    const double TIRE_RADIUS_M = 0.315;
    // 变速箱各档位传动比 (示例值，基于典型6MT分布及思域6挡巡航数据估算[citation:1])
    const double GEAR_RATIOS[] = {3.642, 2.08, 1.361, 1.023, 0.829, 0.686}; // 1-6档
    const int NUM_GEARS = sizeof(GEAR_RATIOS) / sizeof(GEAR_RATIOS[0]);

    // -------- 1. 输入有效性检查 --------
    if (rpm <= 0 || speed_kph <= 0) {
        return 0; // 车辆未移动或发动机未工作
    }

    // -------- 2. 计算当前总传动比 --------
    // 将车速从 km/h 转换为 m/s
    double speed_mps = speed_kph * 1000.0 / 3600.0;
    // 计算车轮转速 (转/秒)，再转换为转/分钟 (RPM)
    double wheel_rpm = (speed_mps / (2 * PI * TIRE_RADIUS_M)) * 60.0;
    // 避免除以零
    if (wheel_rpm <= 0) {
        return -1;
    }
    // 总传动比 = 发动机转速 / 车轮转速
    double current_total_ratio = rpm / wheel_rpm;

    // -------- 3. 推算变速箱档位传动比 --------
    // 变速箱传动比 = 总传动比 / 主减速比
    double current_gear_ratio = current_total_ratio / FINAL_DRIVE_RATIO;

    // -------- 4. 匹配最接近的档位 --------
    int estimated_gear = -1;
    double min_ratio_diff = 1e6; // 初始化为一个大数

    for (int i = 0; i < NUM_GEARS; i++) {
        double diff = fabs(current_gear_ratio - GEAR_RATIOS[i]);
        // 找到与当前计算值最接近的标定传动比
        if (diff < min_ratio_diff) {
            min_ratio_diff = diff;
            estimated_gear = i + 1; // 数组索引0对应1档
        }
    }

    // -------- 5. 置信度检查 --------
    // 如果计算出的传动比与匹配档位的标定值差异过大，则认为结果不可信
    double allowed_error = 0.25; // 允许的误差容限，可根据实际情况调整
    if (min_ratio_diff > allowed_error) {
        return -1; // 传动比异常，可能处于空档、离合器踩下或滑行状态
    }

    return estimated_gear;
}

// ================== 解析函数实现 ==================

// 解析POWERTRAIN_DATA (ID: 380)
void HondaCAN::parsePowertrainData(uint8_t data[8])
{
  Powertrain.PEDAL_GAS = data[0];
  Powertrain.ENGINE_RPM = (uint16_t)(data[2] << 8 | data[3]);
  Powertrain.GAS_PRESSED = (data[4] >> 7) & 0x01;
  Powertrain.ACC_STATUS = (data[4] >> 6) & 0x01;
  Powertrain.BOH_17C = data[4] & 0x1F;
  Powertrain.BRAKE_SWITCH = (data[4] >> 0) & 0x01;
  Powertrain.BOH2_17C = data[5] & 0x03;
  Powertrain.BOH2_17C |= (data[6] & 0xFF) << 2;
  Powertrain.BRAKE_PRESSED = (data[6] >> 5) & 0x01;
  Powertrain.BOH3_17C = (data[6] >> 0) & 0x1F;
}

// 解析ENGINE_DATA (ID: 344)
void HondaCAN::parseEngineData(uint8_t data[8])
{
  EngineData.XMISSION_SPEED = (((uint16_t)data[0] << 8) | data[1]) * 0.01f;
  EngineData.ENGINE_RPM = ((uint16_t)data[2] << 8) | data[3];
  EngineData.XMISSION_SPEED2 = (((uint16_t)data[4] << 8) | data[5]) * 0.01f;
  EngineData.ODOMETER = data[6] * 10;
}

// 解析ENGINE_DATA_3 (需要定义ID)
void HondaCAN::parseEngineData3(uint8_t data[8])
{
  EngineDataThree.ENGINE_TEMP = data[0] - 40;
  EngineDataThree.INTAKE_TEMP = data[1] - 40;
  EngineDataThree.TRIP_FUEL_CONSUMED = (((uint16_t)data[2] << 8) | data[3]) * 0.0001f;
}

// 解析DRIVE_MODES (ID: 545)
void HondaCAN::parseDriveModes(uint8_t data[8])
{
  DriveModes.ECON_ON = ((data[2] >> 0) & 0x01);
}

// 解析VSA_STATUS (ID: 420)
void HondaCAN::parseVsaStatus(uint8_t data[8])
{
  VsaStatus.USER_BRAKE = ((uint16_t)data[0] << 8) | data[1];
  VsaStatus.USER_BRAKE = VsaStatus.USER_BRAKE * 0.015625f - 1.609375f;
  VsaStatus.COMPUTER_BRAKING = (data[2] >> 7) & 0x01;
  VsaStatus.ESP_DISABLED = (data[3] >> 5) & 0x01;
  VsaStatus.BRAKE_HOLD_RELATED = (data[6] >> 4) & 0x01;
  VsaStatus.BRAKE_HOLD_ACTIVE = (data[5] >> 6) & 0x01;
  VsaStatus.BRAKE_HOLD_ENABLED = (data[5] >> 5) & 0x01;
}

// 解析CAR_SPEED (ID: 777)
void HondaCAN::parseCarSpeed(uint8_t data[8])
{
  CarSpeed.ROUGH_CAR_SPEED = data[2];
  CarSpeed.CAR_SPEED = ((uint16_t)data[0] << 8 | data[1]) * 0.01f;
  CarSpeed.ROUGH_CAR_SPEED_3 = (((uint16_t)data[4] << 8) | data[5]) * 0.01f;
  CarSpeed.ROUGH_CAR_SPEED_2 = data[3];
  CarSpeed.LOCK_STATUS = (data[6] >> 6) & 0x03;
  CarSpeed.IMPERIAL_UNIT = (data[7] >> 7) & 0x01;
}

// 解析DOORS_STATUS (ID: 1029)
void HondaCAN::parseDoorsStatus(uint8_t data[8])
{
  DoorsStatus.DOOR_OPEN_FL = (data[4] >> 5) & 0x01;
  DoorsStatus.DOOR_OPEN_FR = (data[4] >> 6) & 0x01;
  DoorsStatus.DOOR_OPEN_RL = (data[4] >> 7) & 0x01;
  DoorsStatus.DOOR_OPEN_RR = (data[5] >> 0) & 0x01;
  DoorsStatus.TRUNK_OPEN = (data[5] >> 1) & 0x01;
}

// ================== 新增解析函数实现 ==================

// 解析GAS_PEDAL_2 (ID: 304)
void HondaCAN::parseGasPedal2(uint8_t data[8])
{
  GasPedal2.ENGINE_TORQUE_ESTIMATE = (int16_t)((data[0] << 8) | data[1]);
  GasPedal2.ENGINE_TORQUE_REQUEST = (int16_t)((data[2] << 8) | data[3]);
  GasPedal2.CAR_GAS = data[4];
}

// 解析STEERING_SENSORS (ID: 330)
void HondaCAN::parseSteeringSensors(uint8_t data[8])
{
  SteeringSensors.STEER_ANGLE = (int16_t)((data[0] << 8) | data[1]) * -0.1f;
  SteeringSensors.STEER_ANGLE_RATE = (int16_t)((data[2] << 8) | data[3]) * -1.0f;
  SteeringSensors.STEER_SENSOR_STATUS_1 = (data[4] >> 2) & 0x01;
  SteeringSensors.STEER_SENSOR_STATUS_2 = (data[4] >> 1) & 0x01;
  SteeringSensors.STEER_SENSOR_STATUS_3 = (data[4] >> 0) & 0x01;
  SteeringSensors.STEER_WHEEL_ANGLE = (int16_t)((data[5] << 8) | data[6]) * -0.1f;

}

// 解析KINEMATICS_ALT (ID: 148)
void HondaCAN::parseKinematicsAlt(uint8_t data[8])
{
  KinematicsAlt.LAT_ACCEL = (int16_t)((data[0] << 8) | data[1]) * 0.02f - 512.0f;
  KinematicsAlt.LONG_ACCEL = (int16_t)((data[2] << 8) | data[3]) * -0.02f;

}

// 解析STANDSTILL (ID: 432)
void HondaCAN::parseStandstill(uint8_t data[8])
{
  Standstill.CONTROLLED_STANDSTILL = data[0] & 0x01;
  Standstill.WHEELS_MOVING = (data[1] >> 4) & 0x01;
  Standstill.BRAKE_ERROR_1 = (data[1] >> 3) & 0x01;
  Standstill.BRAKE_ERROR_2 = (data[1] >> 1) & 0x01;

}

// 解析WHEEL_SPEEDS (ID: 464)
void HondaCAN::parseWheelSpeeds(uint8_t data[8])
{
  WheelSpeeds.WHEEL_SPEED_FL = ((uint16_t)((data[0] << 7) | (data[1] >> 1)) & 0x7FFF) * 0.01f;
  WheelSpeeds.WHEEL_SPEED_FR = ((uint16_t)((data[1] << 7) | (data[2] >> 1)) & 0x7FFF) * 0.01f;
  WheelSpeeds.WHEEL_SPEED_RL = ((uint16_t)((data[3] << 7) | (data[4] >> 1)) & 0x7FFF) * 0.01f;
  WheelSpeeds.WHEEL_SPEED_RR = ((uint16_t)((data[4] << 7) | (data[5] >> 1)) & 0x7FFF) * 0.01f;
}

// 解析VEHICLE_DYNAMICS (ID: 490)
void HondaCAN::parseVehicleDynamics(uint8_t data[8])
{
  VehicleDynamics.LAT_ACCEL = (int16_t)((data[0] << 8) | data[1]) * 0.0015f;
  VehicleDynamics.LONG_ACCEL = (int16_t)((data[2] << 8) | data[3]) * 0.0015f;
}

// 解析STEER_MOTOR_TORQUE (ID: 427)
void HondaCAN::parseSteerMotorTorque(uint8_t data[8])
{
  SteerMotorTorque.CONFIG_VALID = (data[0] >> 7) & 0x01;
  SteerMotorTorque.MOTOR_TORQUE = ((uint16_t)((data[0] << 9) | (data[1] << 1)) & 0x3FF);
  SteerMotorTorque.OUTPUT_DISABLED = (data[2] >> 6) & 0x01;
}

// 解析EPB_STATUS (ID: 450)
void HondaCAN::parseEpbStatus(uint8_t data[8])
{
  EpbStatus.EPB_BRAKE_AND_PULL = (data[0] >> 6) & 0x01;
  EpbStatus.EPB_ACTIVE = (data[0] >> 3) & 0x01;
  EpbStatus.EPB_STATE = (data[3] >> 5) & 0x03;
}

// 解析ECON_STATUS (ID: 545)
void HondaCAN::parseEconStatus(uint8_t data[8])
{
  EconStatus.ECON_ON_2 = (data[4] >> 5) & 0x03;
  EconStatus.ECON_ON = (data[2] >> 7) & 0x01;
}

// 解析ROUGH_WHEEL_SPEED (ID: 597)
void HondaCAN::parseRoughWheelSpeed(uint8_t data[8])
{
  RoughWheelSpeed.WHEEL_SPEED_FL = data[0];
  RoughWheelSpeed.WHEEL_SPEED_FR = data[1];
  RoughWheelSpeed.WHEEL_SPEED_RL = data[2];
  RoughWheelSpeed.WHEEL_SPEED_RR = data[3];
  RoughWheelSpeed.SET_TO_X55 = data[4];
  RoughWheelSpeed.SET_TO_X55_2 = data[5];
  RoughWheelSpeed.LONG_COUNTER = data[6];
}

// 解析SCM_BUTTONS (ID: 662)
void HondaCAN::parseScmButtons(uint8_t data[8])
{
  ScmButtons.CRUISE_BUTTONS = data[0] & 0x07;
  ScmButtons.CRUISE_SETTING = (data[0] >> 3) & 0x03;
}

// 解析SEATBELT_STATUS (ID: 773)
void HondaCAN::parseSeatbeltStatus(uint8_t data[8])
{
  SeatbeltStatus.SEATBELT_DRIVER_LAMP = data[0] & 0x01;
  SeatbeltStatus.SEATBELT_PASS_UNLATCHED = (data[1] >> 2) & 0x01;
  SeatbeltStatus.SEATBELT_PASS_LATCHED = (data[1] >> 3) & 0x01;
  SeatbeltStatus.SEATBELT_DRIVER_UNLATCHED = (data[1] >> 4) & 0x01;
  SeatbeltStatus.SEATBELT_DRIVER_LATCHED = (data[1] >> 5) & 0x01;
  SeatbeltStatus.PASS_AIRBAG_OFF = (data[1] >> 6) & 0x01;
  SeatbeltStatus.PASS_AIRBAG_ON = (data[1] >> 7) & 0x01;
}

// 解析SCM_FEEDBACK (ID: 806)
void HondaCAN::parseScmFeedback(uint8_t data[8])
{
  ScmFeedback.DRIVERS_DOOR_OPEN = (data[2] >> 1) & 0x01;
  ScmFeedback.REVERSE_LIGHT = (data[2] >> 2) & 0x01;
  ScmFeedback.CMBS_BUTTON = (data[2] >> 6) & 0x03;
  ScmFeedback.LEFT_BLINKER = (data[3] >> 2) & 0x01;
  ScmFeedback.RIGHT_BLINKER = (data[3] >> 3) & 0x01;
  ScmFeedback.MAIN_ON = (data[3] >> 4) & 0x01;
  ScmFeedback.PARKING_BRAKE_ON = (data[3] >> 5) & 0x01;
}

// 解析STALK_STATUS (ID: 884)
void HondaCAN::parseStalkStatus(uint8_t data[8])
{
  StalkStatus.DASHBOARD_ALERT = data[4];
  StalkStatus.AUTO_HEADLIGHTS = (data[5] >> 6) & 0x01;
  StalkStatus.HIGH_BEAM_HOLD = (data[5] >> 7) & 0x01;
  StalkStatus.HIGH_BEAM_FLASH = (data[5] >> 5) & 0x01;
  StalkStatus.HEADLIGHTS_ON = (data[6] >> 6) & 0x01;
  StalkStatus.WIPER_SWITCH = (data[6] >> 5) & 0x03;
}

// 解析STALK_STATUS_2 (ID: 891)
void HondaCAN::parseStalkStatus2(uint8_t data[8])
{
  StalkStatus2.WIPERS = (data[2] >> 1) & 0x03;
  StalkStatus2.LOW_BEAMS = (data[4] >> 3) & 0x01;
  StalkStatus2.HIGH_BEAMS = (data[4] >> 2) & 0x01;
  StalkStatus2.PARK_LIGHTS = (data[4] >> 4) & 0x01;
}

// 解析ODOMETER (ID: 1302)
void HondaCAN::parseOdometer(uint8_t data[8])
{
  Odometer.ODOMETER = (uint32_t)((data[0] << 16) | (data[1] << 8) | data[2]);
}

// 解析GEARBOX_CVT (ID: 401)
void HondaCAN::parseGearboxCvt(uint8_t data[8])
{
  GearboxCvt.SELECTED_P = data[0] & 0x01;
  GearboxCvt.SELECTED_R = (data[0] >> 1) & 0x01;
  GearboxCvt.SELECTED_N = (data[0] >> 2) & 0x01;
  GearboxCvt.SELECTED_D = (data[0] >> 3) & 0x01;
  GearboxCvt.FORWARD_DRIVING_MODE = (data[2] >> 7) & 0x01;
  GearboxCvt.CVT_UNKNOWN_1 = data[3];
  GearboxCvt.CVT_UNKNOWN_2 = data[4];
  GearboxCvt.GEAR_SHIFTER = (data[5] >> 4) & 0x0F;
  GearboxCvt.SHIFTER_POSITION_VALID = (data[5] >> 5) & 0x01;
  GearboxCvt.NOT_FORWARD_GEAR = (data[6] >> 0) & 0x01;
  GearboxCvt.CVT_UNKNOWN_3 = (data[6] >> 5) & 0x03;
}

// 解析GEARBOX (ID: 422)
void HondaCAN::parseGearbox(uint8_t data[8])
{
  Gearbox.GEAR_SHIFTER = data[0] & 0x3F;
}

// 解析HUD_SETTING (ID: 493)
void HondaCAN::parseHudSetting(uint8_t data[8])
{
  HudSetting.IMPERIAL_UNIT = (data[0] >> 5) & 0x01;
}

// 解析CRUISE (ID: 804) - 定速巡航数据
void HondaCAN::parseCruiseData(uint8_t data[8])
{
    // HUD_SPEED_KPH: 位7-14 (8 bits)
    CruiseData.HUD_SPEED_KPH = data[0];
    
    // HUD_SPEED_MPH: 位15-22 (8 bits)
    CruiseData.HUD_SPEED_MPH = data[1];
    
    // TRIP_FUEL_CONSUMED: 位23-38 (16 bits)
    CruiseData.TRIP_FUEL_CONSUMED = (uint16_t)((data[2] << 8) | data[3]);
    
    // CRUISE_SPEED_PCM: 位39-46 (8 bits) - 注意：255表示未设置速度
    CruiseData.CRUISE_SPEED_PCM = data[4];
    
    // BOH2: 位47-54 (8 bits) - 有符号值
    CruiseData.BOH2 = (int8_t)data[5];
    
    // BOH3: 位55-62 (8 bits)
    CruiseData.BOH3 = data[6];
}
// ================== OBD2功能 ==================

void HondaCAN::obd2_send(uint8_t mode, uint8_t pid)
{
  Serial.println("Writing Data");
  twai_message_t message;

  if (CAN_BIT == 29)
  {
    message.identifier = 0x18DB33F1;
    message.extd = 1;
  }
  else if (CAN_BIT == 11)
  {
    message.identifier = 0x7DF;
    message.extd = 0;
  }

  message.rtr = 0;              // Data frame
  message.data_length_code = 8; // 8-byte data frame
  message.data[0] = 0x02;       // Query length (2 bytes: Mode and PID)
  message.data[1] = mode;       // Mode 01: Request current data
  message.data[2] = pid;        // PID
  message.data[3] = 0x00;       // Padding bytes
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
  {
    Serial.println("Query sent.");
  }
  else
  {
    Serial.println("Failed to send query!");
  }
}

bool HondaCAN::obd2_receive()
{
  Serial.println("Reading...");
  twai_message_t response;
  if (twai_receive(&response, pdMS_TO_TICKS(2000)) == ESP_OK)
  {
    if (response.identifier == 0x7E9 || response.identifier == 0x7EB || response.identifier == 0x7EA || response.identifier == 0x7E8)
    {
      if (memcmp(&obdResponse, &response, sizeof(twai_message_t)) != 0)
      {
        memcpy(&obdResponse, &response, sizeof(twai_message_t));
      }

      Serial.print("Received Data: ");
      Serial.print("ID: 0x");
      Serial.print(response.identifier, HEX);
      Serial.print(", RTR: ");
      Serial.print(response.rtr, HEX);
      Serial.print(", EID: ");
      Serial.print(response.extd, HEX);
      Serial.print(", (DLC): ");
      Serial.print(response.data_length_code);
      Serial.print(", Data: ");
      for (int i = 0; i < response.data_length_code; i++)
      {
        if (response.data[i] < 10)
        {
          Serial.print("0");
          Serial.print(response.data[i], HEX);
        }
        else
        {
          Serial.print(response.data[i], HEX);
        }

        if (i < response.data_length_code - 1)
        {
          Serial.print(" ");
        }
      }
      Serial.println();
      return true;
    }
  }
  else
  {
    Serial.println("Not Received any Message!");
  }
  return false;
}

float HondaCAN::obd2Request(uint8_t pid)
{
  float result = 0;
  this->obd2_send(read_LiveData, pid);
  if (this->obd2_receive())
  {
    if (obdResponse.data[2] == ENGINE_LOAD)
    {
      result = (100.0 / 255) * obdResponse.data[3];
    }
    else if (obdResponse.data[2] == ENGINE_COOLANT_TEMP)
    {
      result = obdResponse.data[3] - 40;
    }
    else if (obdResponse.data[2] == INTAKE_MANIFOLD_ABS_PRESSURE)
    {
      result = obdResponse.data[3];
    }
    else if (obdResponse.data[2] == ENGINE_RPM)
    {
      result = (256 * obdResponse.data[3] + obdResponse.data[4]) / 4;
    }
    else if (obdResponse.data[2] == VEHICLE_SPEED)
    {
      result = obdResponse.data[3];
    }
    else if (obdResponse.data[2] == INTAKE_AIR_TEMP)
    {
      result = obdResponse.data[3] - 40;
    }
  }
  return result;
}

String HondaCAN::obd2RequestVIN()
{
  String result = "";
  this->obd2_send(read_VehicleInfo, read_VIN);
  if (this->obd2_receive())
  {
    for (int i = 0; i < obdResponse.data_length_code - 2; i++)
    {
      result += (char)obdResponse.data[i + 2];
    }
  }
  return result;
}

// void HondaCAN::checkDTC() {
// uint8_t supportedLiveData[32];
// uint8_t supportedFreezeFrame[32];
// uint8_t supportedVehicleInfo[32];
// String dtcBuffer[32];

//   int dtcs = 0;
//   char dtcBytes[2];

//   writeData(read_DTCs, 0x00);
//   readCAN();

//   int length = sizeof(resultBuffer.data);
//   for (int i = 0; i < length; i++) {
//     dtcBytes[0] = resultBuffer.data[3 + i * 2];
//     dtcBytes[1] = resultBuffer.data[3 + i * 2 + 1];

//     if (dtcBytes[0] == 0 && dtcBytes[1] == 0) {
//       break;
//     } else {
//       String ErrorCode = "";
//       const static char type_lookup[4] = { 'P', 'C', 'B', 'U' };
//       const static char digit_lookup[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

//       ErrorCode += type_lookup[(input_byte1 >> 6) & 0b11];
//       ErrorCode += digit_lookup[(input_byte1 >> 4) & 0b11];
//       ErrorCode += digit_lookup[input_byte1 & 0b1111];
//       ErrorCode += digit_lookup[input_byte2 >> 4];
//       ErrorCode += digit_lookup[input_byte2 & 0b1111];
//       dtcBuffer[dtcs++] = ErrorCode;
//     }
//   }
// }
