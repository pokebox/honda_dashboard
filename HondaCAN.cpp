#include "HondaCAN.h"

HondaCAN::HondaCAN()
{
}

bool HondaCAN::begin(/*uint32_t filter = 0xFFFFFFFF*/)
{
  pinMode(CAN_RS, OUTPUT);
  digitalWrite(CAN_RS, LOW); // LOW = high speed mode, HIGH = low power mode (listen only)

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); //{.acceptance_code = filter, .acceptance_mask = 0x7FF, .single_filter = true};

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    return false;
  if (twai_start() != ESP_OK)
    return false;
  return true;
}

void HondaCAN::run()
{
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    switch (message.identifier)
    {
    case POWERTRAIN_DATA_ID:
      this->parsePowertrainData(message.data);
      break;
    case GEARBOX_ID:
      this->parseGearbox(message.data);
      break;
    case CAR_SPEED_ID:
      this->parseCarSpeed(message.data);
      break;
    case VSA_STATUS_ID:
      this->parseVsaStatus(message.data);
      break;
    case DRIVE_MODES_ID:
      this->parseDriveModes(message.data);
      break;
    case ENGINE_DATA_3_ID:
      this->parseEngineData3(message.data);
      break;
    case ENGINE_DATA_ID:          // 注意：从ENGINE_DATA_2_ID改为ENGINE_DATA_ID
      this->parseEngineData(message.data);
      break;
    case DOORS_STATUS_ID:         // 新增车门状态解析
      this->parseDoorsStatus(message.data);
      break;
    }
  }
}

// 解析POWERTRAIN_DATA (ID: 380)
void HondaCAN::parsePowertrainData(uint8_t data[8])
{
  Powertrain.PEDAL_GAS = data[0];
  Powertrain.ENGINE_RPM = (uint16_t)(data[2] << 8 | data[3]);
  Powertrain.GAS_PRESSED = (data[4] >> 7) & 0x01;
  Powertrain.ACC_STATUS = (data[4] >> 6) & 0x01;        // 新增
  Powertrain.BOH_17C = data[4] & 0x1F;                  // 新增：37|5 bits
  Powertrain.BRAKE_SWITCH = (data[4] >> 0) & 0x01;
  Powertrain.BOH2_17C = data[5] & 0x03;                 // 新增：47|2 bits (实际是10位，但需要跨字节处理)
  Powertrain.BOH2_17C |= (data[6] & 0xFF) << 2;         // 继续处理剩余的8位
  Powertrain.BRAKE_PRESSED = (data[6] >> 5) & 0x01;
  Powertrain.BOH3_17C = (data[6] >> 0) & 0x1F;          // 新增：52|5 bits
  Powertrain.COUNTER = (data[7] >> 6) & 0x03;           // 新增：61|2 bits
  Powertrain.CHECKSUM = (data[7] >> 2) & 0x0F;          // 新增：59|4 bits
}

// 解析ENGINE_DATA (ID: 344) - 原ENGINE_DATA_2
void HondaCAN::parseEngineData(uint8_t data[8]) {
  EngineData.XMISSION_SPEED = (((uint16_t)data[0] << 8) | data[1]) * 0.01f;
  EngineData.ENGINE_RPM = ((uint16_t)data[2] << 8) | data[3];  // 新增
  EngineData.XMISSION_SPEED2 = (((uint16_t)data[4] << 8) | data[5]) * 0.01f;  // 新增
  EngineData.ODOMETER = data[6] * 10;                         // 新增：单位是10米
  EngineData.COUNTER = (data[7] >> 6) & 0x03;                 // 新增：61|2 bits
  EngineData.CHECKSUM = (data[7] >> 2) & 0x0F;                // 新增：59|4 bits
}

void HondaCAN::parseEngineData3(uint8_t data[8]) {
  EngineDataThree.ENGINE_TEMP = data[0] - 40;
  EngineDataThree.INTAKE_TEMP = data[1] - 40;
  EngineDataThree.TRIP_FUEL_CONSUMED = (((uint16_t)data[2] << 8) | data[3]) * 0.0001f;
}

void HondaCAN::parseDriveModes(uint8_t data[8]) {
  DriveModes.ECON_ON = ((data[2] >> 0) & 0x01);
}

// 解析VSA_STATUS (ID: 420)
void HondaCAN::parseVsaStatus(uint8_t data[8])
{
  VsaStatus.USER_BRAKE = ((uint16_t)data[0] << 8) | data[1];
  VsaStatus.COMPUTER_BRAKING = (data[2] >> 7) & 0x01;          // 新增：23|1 bit
  VsaStatus.ESP_DISABLED = (data[3] >> 5) & 0x01;              // 28|1 bit
  VsaStatus.BRAKE_HOLD_RELATED = (data[6] >> 4) & 0x01;        // 新增：52|1 bit
  VsaStatus.BRAKE_HOLD_ACTIVE = (data[5] >> 6) & 0x01;         // 新增：46|1 bit
  VsaStatus.BRAKE_HOLD_ENABLED = (data[5] >> 5) & 0x01;        // 新增：45|1 bit
  VsaStatus.COUNTER = (data[7] >> 6) & 0x03;                   // 新增：61|2 bits
  VsaStatus.CHECKSUM = (data[7] >> 2) & 0x0F;                  // 新增：59|4 bits
}

void HondaCAN::parseGearbox(uint8_t data[8])
{
  Gearbox.GEAR_SHIFTER = data[0] & 0x3F;
}

void HondaCAN::parseCarSpeed(uint8_t data[8])
{
  CarSpeed.ROUGH_CAR_SPEED_3 = (((data[4] << 8) | data[5]) & 0xFFFF) * 0.01f; // (39-54)
  // CarSpeed.LOCK_STATUS = (data[6] >> 6) & 0x03;
  // CarSpeed.IMPERIAL_UNIT = (data[7] >> 7) & 0x01;
}

// void deviceVoltage(void) {
//   return analogRead(SENSE_V_ANA);
// }

// 新增：解析DOORS_STATUS (ID: 1029)
void HondaCAN::parseDoorsStatus(uint8_t data[8])
{
  DoorsStatus.DOOR_OPEN_FL = (data[4] >> 5) & 0x01;     // 37|1 bit
  DoorsStatus.DOOR_OPEN_FR = (data[4] >> 6) & 0x01;     // 38|1 bit
  DoorsStatus.DOOR_OPEN_RL = (data[4] >> 7) & 0x01;     // 39|1 bit
  DoorsStatus.DOOR_OPEN_RR = (data[5] >> 0) & 0x01;     // 40|1 bit
  DoorsStatus.TRUNK_OPEN = (data[5] >> 1) & 0x01;       // 41|1 bit
  DoorsStatus.COUNTER = (data[7] >> 6) & 0x03;          // 61|2 bits
  DoorsStatus.CHECKSUM = (data[7] >> 2) & 0x0F;         // 59|4 bits
}


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
