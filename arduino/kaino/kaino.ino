#include <Arduino.h>
#include <math.h>
#include "MGDrive.h"
#include <ArduinoJson.h>

// setup master can id and motor can id (default cybergear can id is 0x7F)
#define MAX_MGID 4
// Define your CS and INT pins
// init MCP_CAN object
MCP_CAN CAN0(9);
// MCP_CAN CAN1(10);

#define CAN0_INT 2
// #define CAN1_INT 3

MGDrive mgDrive_wheel(&CAN0, CAN0_INT);
// MGDrive mgDrive_str(&CAN1, CAN1_INT);

float mymap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float rpmtoradps(float rpm) {
  return ((rpm * 2 * PI) / 60) * 6;
}

float radpstorpm(float radps) {
  return (radps / 6) * (30 / PI);
}

#define MYID 1

void setup() {
  SerialUSB.begin(115200);

  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    SerialUSB.print("CAN0: Init OK!\r\n");
    CAN0.setMode(MCP_NORMAL);

  } else SerialUSB.print("CAN0: Init Fail!!!\r\n");

  // if (CAN1.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
  //   SerialUSB.print("CAN1: Init OK!\r\n");
  //   CAN1.setMode(MCP_NORMAL);
  // } else SerialUSB.print("CAN1: Init Fail!!!\r\n");

  mgDrive_wheel.Motor_stop(MYID);
  // mgDrive_wheel.Motor_stop(4);

  mgDrive_wheel.Clearmotorerrorstate(MYID);
  // mgDrive_wheel.Clearmotorerrorstate(4);
  // mgDrive_str.Clearmotorerrorstate(5);
  // mgDrive_str.Clearmotorerrorstate(6);

  mgDrive_wheel.Motor_on(MYID);
  // mgDrive_wheel.Motor_on(4);
  // mgDrive_str.Motor_on(5);
  // mgDrive_str.Motor_on(6);
}

static unsigned long millis_pre = 0;
static unsigned long millis_pre_1 = 0;
static unsigned long millis_now = millis();

void loop() {
  millis_now = millis();
  if ((millis_now - millis_pre) > 5000) {
    millis_pre = millis_now;

    static int i = 90;
    SerialUSB.println(i);
    // mgDrive_wheel.Multimotortorqueclosedloopcontrol(0.0, 0.5, 0.0, 0.5);
    // mgDrive_wheel.Torqueclosedloopcontrol(2, 0.4);
    // mgDrive_wheel.Torqueclosedloopcontrol(4, 0.4);
    // mgDrive_wheel.Motor_stop(2);
    // mgDrive_wheel.Motor_stop(4);
    mgDrive_wheel.Speedclosedloopcontrol(MYID, 0.7);
    // mgDrive_wheel.Speedclosedloopcontrol(4, 60);
    // mgDrive_str.Multiloopanglecontrol2(5, 6, i);
    // mgDrive_str.Multiloopanglecontrol2(6, 6, i);

    switch (i) {
      case 0:
        i = 90;
        break;
      case 90:
        i = 180;
        break;
      case 180:
        i = 0;
        break;
      default:
        i = 0;
        break;
    }
  } else if ((millis_now - millis_pre_1) > 1000) {
    millis_pre_1 = millis_now;

    const int capacity = JSON_ARRAY_SIZE(MAX_MGID) + MAX_MGID * JSON_OBJECT_SIZE(5);
    StaticJsonDocument<capacity> doc;
    static char char_mg_msg[capacity];

    mgDrive_wheel.ReadMotorState2(MYID);
    // mgDrive_wheel.ReadMotorState2(4);
    // mgDrive_str.ReadMotorState2(5);
    // mgDrive_str.ReadMotorState2(6);

    for (int i = 2; i <= 6; i++) {
      // if (i <= 4 && i != 3) {
      //   SerialUSB.print("MG ID: ");
      //   SerialUSB.print(i);
      //   SerialUSB.print(" Temperature: ");
      //   SerialUSB.print(mgDrive_wheel.getTemperatureValue(i));
      //   SerialUSB.print(" TorqueCurrent: ");
      //   SerialUSB.print(mgDrive_wheel.getTorqueCurrentValue(i));
      //   SerialUSB.print(" MotorSpeed: ");
      //   SerialUSB.print(mgDrive_wheel.getSpeedValue(i));
      //   SerialUSB.print(" EncoderPosition: ");
      //   SerialUSB.println(mgDrive_wheel.getEncoderPositionValue(i));

      // } else if (i != 3) {
      //   SerialUSB.print("MG ID: ");
      //   SerialUSB.print(i);
      //   SerialUSB.print(" Temperature: ");
      //   SerialUSB.print(mgDrive_str.getTemperatureValue(i));
      //   SerialUSB.print(" TorqueCurrent: ");
      //   SerialUSB.print(mgDrive_str.getTorqueCurrentValue(i));
      //   SerialUSB.print(" MotorSpeed: ");
      //   SerialUSB.print(mgDrive_str.getSpeedValue(i));
      //   SerialUSB.print(" EncoderPosition: ");
      //   SerialUSB.println(mgDrive_str.getEncoderPositionValue(i));
      // }
    }
    SerialUSB.print("MG ID: ");
    SerialUSB.println(MYID);
    SerialUSB.print("Temperature: ");
    SerialUSB.println(mgDrive_wheel.getTemperatureValue(MYID));
    SerialUSB.print("TorqueCurrent: ");
    SerialUSB.println(mgDrive_wheel.getTorqueCurrentValue(MYID));
    SerialUSB.print("MotorSpeed: ");
    SerialUSB.println(mgDrive_wheel.getSpeedValue(MYID));
    SerialUSB.print("EncoderPosition: ");
    SerialUSB.println(mgDrive_wheel.getEncoderPositionValue(MYID));

    // SerialUSB.print("MG ID: 4");
    // SerialUSB.print(" Temperature: ");
    // SerialUSB.print(mgDrive_wheel.getTemperatureValue(4));
    // SerialUSB.print(" TorqueCurrent: ");
    // SerialUSB.print(mgDrive_wheel.getTorqueCurrentValue(4));
    // SerialUSB.print(" MotorSpeed: ");
    // SerialUSB.print(mgDrive_wheel.getSpeedValue(4));
    // SerialUSB.print(" EncoderPosition: ");
    // SerialUSB.println(mgDrive_wheel.getEncoderPositionValue(4));

    // SerialUSB.print("MG ID: 5");
    // SerialUSB.print(" Temperature: ");
    // SerialUSB.print(mgDrive_str.getTemperatureValue(5));
    // SerialUSB.print(" TorqueCurrent: ");
    // SerialUSB.print(mgDrive_str.getTorqueCurrentValue(5));
    // SerialUSB.print(" MotorSpeed: ");
    // SerialUSB.print(mgDrive_str.getSpeedValue(5));
    // SerialUSB.print(" EncoderPosition: ");
    // SerialUSB.println(mgDrive_str.getEncoderPositionValue(5));

    serializeJson(doc, char_mg_msg, sizeof(char_mg_msg));
    // SerialUSB.println(char_mg_msg);

    // if (driver.read_pid_parameter()) {
    //   motor_parameter = driver.motor_parameter();
    //   SerialUSB.println("speed_kp=" + String(motor_parameter.speed_kp) + " speed_ki=" + String(motor_parameter.speed_ki) + " angle_ki=" + String(motor_parameter.angle_ki) + " angle_kp=" + String(motor_parameter.angle_kp) + " iq_ki=" + String(motor_parameter.iq_ki) + " iq_kp=" + String(motor_parameter.iq_kp));
    //   driver.write_pid_parameter_to_ram(50.0, 0.0, 1.0, 0.0, 50.0, 0.0);
    // }
    // delay(5000);
  }
}
