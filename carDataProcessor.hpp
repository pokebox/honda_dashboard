#include "honda_data.pb-c.h"
#include "HondaCAN.h"

class CarDataProcessor {
private:
    HondaCAN& CAN;
    //static MessageHeader msg_header;
    CarStatus car_data;
    PCMDATA pcm_data;
    VSADATA vsa_data;
    EPSDATA eps_data;
    OTHERDATA other_data;
    ALLDATA all_data;

    PowertrainData powertrain_data;
    EngineData engine_data;
    GasPedal2 gas_pedal2;
    SteeringSensors steering_sensors;
    KinematicsAlt kinematics_alt;
    VsaStatus vsa_status;
    Standstill standstill;
    WheelSpeeds wheel_speeds;
    VehicleDynamics vehicle_dynamics;
    SteerMotorTorque steer_motor_torque;
    EpbStatus epb_status;
    EconStatus econ_status;
    RoughWheelSpeed rough_wheel_speed;
    ScmButtons scm_buttons;
    SeatbeltStatus seatbelt_status;
    CarSpeed car_speed;
    ScmFeedback scm_feedback;
    StalkStatus stalk_status;
    StalkStatus2 stalk_status2;
    DoorsStatus doors_status;
    Odometer odometer;
    EngineData3 engine_data3;
    Gearbox gearbox;
public:
    CarDataProcessor(HondaCAN& handler) : CAN(handler) {
        // msg_header = MESSAGE_HEADER__INIT;
        car_data = CAR_STATUS__INIT;
        pcm_data = PCM__DATA__INIT;
        vsa_data = VSA__DATA__INIT;
        eps_data = EPS__DATA__INIT;
        other_data = OTHER__DATA__INIT;
        all_data = ALL__DATA__INIT;

        powertrain_data = POWERTRAIN_DATA__INIT;
        engine_data = ENGINE_DATA__INIT;
        gas_pedal2 = GAS_PEDAL2__INIT;
        steering_sensors = STEERING_SENSORS__INIT;
        kinematics_alt = KINEMATICS_ALT__INIT;
        vsa_status = VSA_STATUS__INIT;
        standstill = STANDSTILL__INIT;
        wheel_speeds = WHEEL_SPEEDS__INIT;
        vehicle_dynamics = VEHICLE_DYNAMICS__INIT;
        steer_motor_torque = STEER_MOTOR_TORQUE__INIT;
        epb_status = EPB_STATUS__INIT;
        econ_status = ECON_STATUS__INIT;
        rough_wheel_speed = ROUGH_WHEEL_SPEED__INIT;
        scm_buttons = SCM_BUTTONS__INIT;
        seatbelt_status = SEATBELT_STATUS__INIT;
        car_speed = CAR_SPEED__INIT;
        scm_feedback = SCM_FEEDBACK__INIT;
        stalk_status = STALK_STATUS__INIT;
        stalk_status2 = STALK_STATUS2__INIT;
        doors_status = DOORS_STATUS__INIT;
        odometer = ODOMETER__INIT;
        engine_data3 = ENGINE_DATA3__INIT;
        gearbox = GEARBOX__INIT;
        setupCarStatusPointers();
    }

    void setupCarStatusPointers() {
        // 设置枚举类型，表示哪个字段被设置
        car_data.powertrain_data_case = CAR_STATUS__POWERTRAIN_DATA_POWERTRAIN;
        car_data.engine_data_case = CAR_STATUS__ENGINE_DATA_ENGINE;
        car_data.gas_pedal_2_data_case = CAR_STATUS__GAS_PEDAL_2_DATA_GAS_PEDAL2;
        car_data.steering_data_case = CAR_STATUS__STEERING_DATA_STEERING_SENSORS;
        car_data.kinematics_data_case = CAR_STATUS__KINEMATICS_DATA_KINEMATICS_ALT;
        car_data.vsa_status_data_case = CAR_STATUS__VSA_STATUS_DATA_VSA_STATUS;
        car_data.standstill_data_case = CAR_STATUS__STANDSTILL_DATA_STANDSTILL;
        car_data.wheel_speeds_data_case = CAR_STATUS__WHEEL_SPEEDS_DATA_WHEELS_SPEEDS;
        car_data.vehicle_dynamics_data_case = CAR_STATUS__VEHICLE_DYNAMICS_DATA_DYNAMICS;
        car_data.steer_motor_torque_data_case = CAR_STATUS__STEER_MOTOR_TORQUE_DATA_STEER_MOTOR_TORQUE;
        car_data.epb_status_data_case = CAR_STATUS__EPB_STATUS_DATA_EPB_STATUS;
        car_data.econ_status_data_case = CAR_STATUS__ECON_STATUS_DATA_ECON_STATUS;
        car_data.rough_wheel_speed_data_case = CAR_STATUS__ROUGH_WHEEL_SPEED_DATA_ROUGH_WHEEL_SPEED;
        car_data.scm_buttons_data_case = CAR_STATUS__SCM_BUTTONS_DATA_SCM_BUTTONS;
        car_data.seatbelt_status_data_case = CAR_STATUS__SEATBELT_STATUS_DATA_SEATBELT_STATUS;
        car_data.car_speed_data_case = CAR_STATUS__CAR_SPEED_DATA_CAR_SPEED;
        car_data.scm_feedback_data_case = CAR_STATUS__SCM_FEEDBACK_DATA_SCM_FEEDBACK;
        car_data.stalk_status_data_case = CAR_STATUS__STALK_STATUS_DATA_STALK_STATUS;
        car_data.stalk_status_2_data_case = CAR_STATUS__STALK_STATUS_2_DATA_STALK_STATUS2;
        car_data.doors_status_data_case = CAR_STATUS__DOORS_STATUS_DATA_DOORS_STATUS;
        car_data.odometer_data_case = CAR_STATUS__ODOMETER_DATA_ODOMETER;
        car_data.engine_data_3_case = CAR_STATUS__ENGINE_DATA_3_ENGINE_DATA3;

        pcm_data.gaspedal2 = &gas_pedal2;
        pcm_data.enginedata = &engine_data;
        pcm_data.powertraindata = &powertrain_data;
        pcm_data.carspeed = &car_speed;
        pcm_data.enginedata3 = &engine_data3;
        pcm_data.gearbox = &gearbox;

        vsa_data.vsastatus = &vsa_status;
        vsa_data.wheelsspeeds = &wheel_speeds;
        vsa_data.vehicledynamics = &vehicle_dynamics;
        vsa_data.roughwheelspeed = &rough_wheel_speed;
        vsa_data.standstill = &standstill;

        eps_data.steeringsensors = &steering_sensors;
        eps_data.steermotortorque = &steer_motor_torque;

        other_data.epbstatus = &epb_status;
        other_data.econstatus = &econ_status;
        other_data.scmfeedback = &scm_feedback;
        other_data.stalkstatus = &stalk_status;
        other_data.stalkstatus2 = &stalk_status2;
        other_data.doorsstatus = &doors_status;
        other_data.odometer = &odometer;
        other_data.scmbuttons = &scm_buttons;
        other_data.seatbeltstatus = &seatbelt_status;

        all_data.pcmdata = &pcm_data;
        all_data.vsadata = &vsa_data;
        all_data.epsdata = &eps_data;
        all_data.otherdata = &other_data;
        
        // 设置实际的指针字段
        car_data.powertrain = &powertrain_data;
        car_data.engine = &engine_data;
        car_data.gaspedal2 = &gas_pedal2;
        car_data.steeringsensors = &steering_sensors;
        car_data.kinematicsalt = &kinematics_alt;
        car_data.vsastatus = &vsa_status;
        car_data.standstill = &standstill;
        car_data.wheelsspeeds = &wheel_speeds;
        car_data.dynamics = &vehicle_dynamics;
        car_data.steermotortorque = &steer_motor_torque;
        car_data.epbstatus = &epb_status;
        car_data.econstatus = &econ_status;
        car_data.roughwheelspeed = &rough_wheel_speed;
        car_data.scmbuttons = &scm_buttons;
        car_data.seatbeltstatus = &seatbelt_status;
        car_data.carspeed = &car_speed;
        car_data.scmfeedback = &scm_feedback;
        car_data.stalkstatus = &stalk_status;
        car_data.stalkstatus2 = &stalk_status2;
        car_data.doorsstatus = &doors_status;
        car_data.odometer = &odometer;
        car_data.enginedata3 = &engine_data3;
    }

    void updateData() {
        powertrain_data.pedal_gas = CAN.Powertrain.PEDAL_GAS;
        powertrain_data.engine_rpm = CAN.Powertrain.ENGINE_RPM;
        powertrain_data.gas_pressed = CAN.Powertrain.GAS_PRESSED;
        powertrain_data.acc_status = CAN.Powertrain.ACC_STATUS;
        powertrain_data.boh_17c = CAN.Powertrain.BOH_17C;
        powertrain_data.brake_switch = CAN.Powertrain.BRAKE_SWITCH;
        powertrain_data.boh2_17c = CAN.Powertrain.BOH2_17C;
        powertrain_data.brake_pressed = CAN.Powertrain.BRAKE_PRESSED;
        powertrain_data.boh3_17c = CAN.Powertrain.BOH3_17C;

        engine_data.xmission_speed = CAN.EngineData.XMISSION_SPEED;
        engine_data.engine_rpm = CAN.EngineData.ENGINE_RPM;
        engine_data.xmission_speed2 = CAN.EngineData.XMISSION_SPEED2;
        engine_data.odometer = CAN.EngineData.ODOMETER;

        gas_pedal2.engine_torque_estimate = CAN.GasPedal2.ENGINE_TORQUE_ESTIMATE;
        gas_pedal2.engine_torque_request = CAN.GasPedal2.ENGINE_TORQUE_REQUEST;
        gas_pedal2.car_gas = CAN.GasPedal2.CAR_GAS;

        steering_sensors.steer_angle = CAN.SteeringSensors.STEER_ANGLE;
        steering_sensors.steer_angle_rate = CAN.SteeringSensors.STEER_ANGLE_RATE;
        steering_sensors.steer_sensor_status_1 = CAN.SteeringSensors.STEER_SENSOR_STATUS_1;
        steering_sensors.steer_sensor_status_2 = CAN.SteeringSensors.STEER_SENSOR_STATUS_2;
        steering_sensors.steer_sensor_status_3 = CAN.SteeringSensors.STEER_SENSOR_STATUS_3;
        steering_sensors.steer_wheel_angle = CAN.SteeringSensors.STEER_WHEEL_ANGLE;

        kinematics_alt.lat_accel = CAN.KinematicsAlt.LAT_ACCEL;
        kinematics_alt.long_accel = CAN.KinematicsAlt.LONG_ACCEL;

        vsa_status.user_brake = CAN.VsaStatus.USER_BRAKE;
        vsa_status.computer_braking = CAN.VsaStatus.COMPUTER_BRAKING;
        vsa_status.esp_disabled = CAN.VsaStatus.ESP_DISABLED;
        vsa_status.brake_hold_related = CAN.VsaStatus.BRAKE_HOLD_RELATED;
        vsa_status.brake_hold_active = CAN.VsaStatus.BRAKE_HOLD_ACTIVE;
        vsa_status.brake_hold_enabled = CAN.VsaStatus.BRAKE_HOLD_ENABLED;

        standstill.controlled_standstill = CAN.Standstill.CONTROLLED_STANDSTILL;
        standstill.wheels_moving = CAN.Standstill.WHEELS_MOVING;
        standstill.brake_error_1 = CAN.Standstill.BRAKE_ERROR_1;
        standstill.brake_error_2 = CAN.Standstill.BRAKE_ERROR_2;

        wheel_speeds.wheel_speed_fl = CAN.WheelSpeeds.WHEEL_SPEED_FL;
        wheel_speeds.wheel_speed_fr = CAN.WheelSpeeds.WHEEL_SPEED_FR;
        wheel_speeds.wheel_speed_rl = CAN.WheelSpeeds.WHEEL_SPEED_RL;
        wheel_speeds.wheel_speed_rr = CAN.WheelSpeeds.WHEEL_SPEED_RR;

        vehicle_dynamics.lat_accel = CAN.VehicleDynamics.LAT_ACCEL;
        vehicle_dynamics.long_accel = CAN.VehicleDynamics.LONG_ACCEL;

        steer_motor_torque.config_valid = CAN.SteerMotorTorque.CONFIG_VALID;
        steer_motor_torque.motor_torque = CAN.SteerMotorTorque.MOTOR_TORQUE;
        steer_motor_torque.output_disabled = CAN.SteerMotorTorque.OUTPUT_DISABLED;

        epb_status.epb_brake_and_pull = CAN.EpbStatus.EPB_BRAKE_AND_PULL;
        epb_status.epb_active = CAN.EpbStatus.EPB_ACTIVE;
        //epb_status.epb_state = CAN.EpbStatus.EPB_STATE;

        econ_status.econ_on_2 = CAN.EconStatus.ECON_ON_2;
        econ_status.econ_on = CAN.EconStatus.ECON_ON;

        rough_wheel_speed.wheel_speed_fl = CAN.RoughWheelSpeed.WHEEL_SPEED_FL;
        rough_wheel_speed.wheel_speed_fr = CAN.RoughWheelSpeed.WHEEL_SPEED_FR;
        rough_wheel_speed.wheel_speed_rl = CAN.RoughWheelSpeed.WHEEL_SPEED_RL;
        rough_wheel_speed.wheel_speed_rr = CAN.RoughWheelSpeed.WHEEL_SPEED_RR;
        rough_wheel_speed.set_to_x55 = CAN.RoughWheelSpeed.SET_TO_X55;
        rough_wheel_speed.set_to_x55_2 = CAN.RoughWheelSpeed.SET_TO_X55_2;
        rough_wheel_speed.long_counter = CAN.RoughWheelSpeed.LONG_COUNTER;

        //scm_buttons.cruise_buttons = CAN.ScmButtons.CRUISE_BUTTONS;
        scm_buttons.cruise_setting = CAN.ScmButtons.CRUISE_SETTING;

        seatbelt_status.seatbelt_driver_lamp = CAN.SeatbeltStatus.SEATBELT_DRIVER_LAMP;
        seatbelt_status.seatbelt_pass_unlatched = CAN.SeatbeltStatus.SEATBELT_PASS_UNLATCHED;
        seatbelt_status.seatbelt_pass_latched = CAN.SeatbeltStatus.SEATBELT_PASS_LATCHED;
        seatbelt_status.seatbelt_driver_unlatched = CAN.SeatbeltStatus.SEATBELT_DRIVER_UNLATCHED;
        seatbelt_status.seatbelt_driver_latched = CAN.SeatbeltStatus.SEATBELT_DRIVER_LATCHED;
        seatbelt_status.pass_airbag_off = CAN.SeatbeltStatus.PASS_AIRBAG_OFF;
        seatbelt_status.pass_airbag_on = CAN.SeatbeltStatus.PASS_AIRBAG_ON;

        car_speed.rough_car_speed = CAN.CarSpeed.ROUGH_CAR_SPEED;
        car_speed.rough_car_speed_2 = CAN.CarSpeed.ROUGH_CAR_SPEED_2;
        car_speed.rough_car_speed_3 = CAN.CarSpeed.ROUGH_CAR_SPEED_3;
        car_speed.car_speed = CAN.CarSpeed.CAR_SPEED;
        car_speed.lock_status = CAN.CarSpeed.LOCK_STATUS;
        car_speed.imperial_unit = CAN.CarSpeed.IMPERIAL_UNIT;

        scm_feedback.drivers_door_open = CAN.ScmFeedback.DRIVERS_DOOR_OPEN;
        scm_feedback.reverse_light = CAN.ScmFeedback.REVERSE_LIGHT;
        //scm_feedback.cmbs_button = CAN.ScmFeedback.CMBS_BUTTON;
        scm_feedback.left_blinker = CAN.ScmFeedback.LEFT_BLINKER;
        scm_feedback.right_blinker = CAN.ScmFeedback.RIGHT_BLINKER;
        scm_feedback.main_on = CAN.ScmFeedback.MAIN_ON;
        scm_feedback.parking_brake_on = CAN.ScmFeedback.PARKING_BRAKE_ON;

        stalk_status.dashboard_alert = CAN.StalkStatus.DASHBOARD_ALERT;
        stalk_status.auto_headlights = CAN.StalkStatus.AUTO_HEADLIGHTS;
        stalk_status.high_beam_hold = CAN.StalkStatus.HIGH_BEAM_HOLD;
        stalk_status.high_beam_flash = CAN.StalkStatus.HIGH_BEAM_FLASH;
        stalk_status.headlights_on = CAN.StalkStatus.HEADLIGHTS_ON;
        stalk_status.wiper_switch = CAN.StalkStatus.WIPER_SWITCH;

        stalk_status2.wipers = CAN.StalkStatus2.WIPERS;
        stalk_status2.low_beams = CAN.StalkStatus2.LOW_BEAMS;
        stalk_status2.high_beams = CAN.StalkStatus2.HIGH_BEAMS;
        stalk_status2.park_lights = CAN.StalkStatus2.PARK_LIGHTS;

        doors_status.door_open_fl = CAN.DoorsStatus.DOOR_OPEN_FL;
        doors_status.door_open_fr = CAN.DoorsStatus.DOOR_OPEN_FR;
        doors_status.door_open_rl = CAN.DoorsStatus.DOOR_OPEN_RL;
        doors_status.door_open_rr = CAN.DoorsStatus.DOOR_OPEN_RR;
        doors_status.trunk_open = CAN.DoorsStatus.TRUNK_OPEN;

        odometer.odometer = CAN.Odometer.ODOMETER;

        engine_data3.engine_temp = CAN.EngineDataThree.ENGINE_TEMP;
        engine_data3.intake_temp = CAN.EngineDataThree.INTAKE_TEMP;
        engine_data3.trip_fuel_consumed = CAN.EngineDataThree.TRIP_FUEL_CONSUMED;

        gearbox.gear_shifter = CAN.Gearbox.GEAR_SHIFTER;
    }

    CarStatus *getCarStatus() {
        return &car_data;
    }

    PCMDATA *getPcmData() {
        return &pcm_data;
    }
    VSADATA *getVsaData() {
        return &vsa_data;
    }
    EPSDATA *getEpsData() {
        return &eps_data;
    }
    OTHERDATA *getOtherData() {
        return &other_data;
    }
    ALLDATA *getAllData() {
        return &all_data;
    }
};
