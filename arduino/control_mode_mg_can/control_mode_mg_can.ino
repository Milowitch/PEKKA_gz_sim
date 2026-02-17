#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
//#include <M5Stack.h>/
#include "lkm_driver_defs.hh"
#include "lkm_driver.hh"
#include <SCServo.h>

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#define INC_POSITION 20.0
#define INC_VELOCITY 1.0
#define INC_TORQUE 0.04
#define MODE_POSITION 0x01
#define MODE_SPEED 0x02
#define MODE_CURRENT 0x03

#define MAX_MGID 4

int Effort[MAX_MGID];
int Position[MAX_MGID];
int Tempareture[MAX_MGID];
int Velocity[MAX_MGID];

float mg_array_msg[4][4];
int mg_array_msg_size = sizeof(mg_array_msg) / sizeof(mg_array_msg[0]);

int32_t sm_array_msg[4][6];
int sm_array_msg_size = sizeof(sm_array_msg) / sizeof(sm_array_msg[0]);

ros::NodeHandle nh;

std_msgs::String mg_msg;
ros::Publisher mg_msg_pub("/mg_msg/rawdata", &mg_msg);
char char_mg_msg[200];

std_msgs::String sm_msg;
ros::Publisher sm_msg_pub("/sm_msg/rawdata", &sm_msg);
char char_sm_msg[200];

// init MCP_CAN object
//#define CAN0_INT 2  // Set INT to pin 2
MCP_CAN CAN0(9);  // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID[] = { 0x01, 0x02, 0x03, 0x04 };
String mg_jsonString;

lkm_m5::Driver driver[4] = { lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[0], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT), lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[1], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT), lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[2], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT), lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[3], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT) };
lkm_m5::MotorState motor_status[4];

uint8_t mode = MODE_SPEED;       //!< current mode
float target_pos = 0.0;          //!< motor target position
float target_vel = 0.0;          //!< motor target velocity
float target_torque = 0.0;       //!< motor target torque
float dir = 1.0f;                //!< direction for motion mode
float default_kp = 50.0f;        //!< default kp for motion mode
float default_kd = 1.0f;         //!< default kd for motion mode
float init_speed = 30.0f;        //!< initial speed
float slow_speed = 1.0f;         //!< slow speed
bool state_change_flag = false;  //!< state change flag

bool speeds_cmd = false;
bool forword = true;
uint32_t speeds = 0;

SCSCL sc;

byte ID[] = { 3, 6, 9, 17 };
uint16_t Position_sc[4];
uint16_t Speeds[] = { 1500, 1500, 1500, 1500 };
String sm_jsonString;
#define MAX_CID 4

int Pos[MAX_CID];
int Speed[MAX_CID];
int Load[MAX_CID];
int Voltage[MAX_CID];
int Temper[MAX_CID];
int Current[MAX_CID];

float mymap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drive(float linear_x = 0.0){
  // target_vel = INC_VELOCITY * mymap(cmd_vel_msg_data.linear.y, 1.0, -1.0, 1.0, 100.0);
  if (linear_x > 0.1) {
    speeds_cmd = true;
  } else if (linear_x < -0.1) {
    speeds_cmd = true;
  } else if (linear_x >= -0.1 && linear_x <= 0.1) {
    speeds_cmd = false;
    driver[0].stop_motor();
  }

  target_vel = linear_x;

  if (mode == MODE_POSITION) {
    // set limit speed when state changed
    driver[0].multi_loop_angle_with_speed_control(target_pos, slow_speed);
  } else if (mode == MODE_SPEED && speeds_cmd) {
    driver[0].speed_closed_loop_control(target_vel);
  } else if (mode == MODE_CURRENT) {
    if (driver[0].motor_type() != MOTOR_SERIES_MS) {
      driver[0].torque_closed_loop_control(target_torque);
    } else {
      driver[0].open_loop_control(target_torque * 100);
    }
  }

  // Position_sc[0] = mymap(cmd_vel_msg_data.angular.z, 1.0, -1.0, 20.0, 1003.0);
  // Position_sc[1] = mymap(cmd_vel_msg_data.angular.z, 1.0, -1.0, 20.0, 1003.0);
  // Position_sc[2] = mymap(cmd_vel_msg_data.angular.z, 1.0, -1.0, 20.0, 1003.0);
  // Position_sc[3] = mymap(cmd_vel_msg_data.angular.z, 1.0, -1.0, 20.0, 1003.0);
  // sc.SyncWritePos(ID, 4, Position_sc, 0, Speeds);
}

void cmd_vel_command(const geometry_msgs::Twist& cmd_vel_msg_data) {
  drive(cmd_vel_msg_data.linear.x);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_command);

#define SERIAL true

void setup() {
  if (SERIAL) {
    Serial.begin(115200);
  } else {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    // nh.advertise(mg_msg_pub);
    // nh.advertise(sm_msg_pub);
  }

  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    if (SERIAL) {
      Serial.println("MCP2515 Initialized Successfully!");
    } else {
      // nh.loginfo("MCP2515 Initialized Successfully!");
    }
  } else {
    if (SERIAL) {
      Serial.println("Error Initializing MCP2515...");
    } else {
      // nh.loginfo("Error Initializing MCP2515...");
    }
  }
  CAN0.setMode(MCP_NORMAL);

  for (char i = 0; i < MAX_MGID; i++) {
    driver[i].init(&CAN0);
    driver[i].motor_on();
  }
  // Serial1.begin(1000000);
  // sc.pSerial = &Serial1;

  if (SERIAL) {
    Serial.println("Setup Done");
  }
}

void loop() {
  if (SERIAL) {
    mode = MODE_SPEED;
    speeds_cmd = true;
    target_vel = 300.0;
    driver[0].speed_closed_loop_control(target_vel);
    // driver[0].motor_stop();
  } else {
  }
  for (char i = 0; i < MAX_MGID; i++) {
    if (driver[i].process_can_packet()) {
      motor_status[i] = driver[i].motor_state();
      mg_array_msg[i][0] = motor_status[i].effort;
      mg_array_msg[i][1] = motor_status[i].position;
      mg_array_msg[i][2] = motor_status[i].tempareture;
      mg_array_msg[i][3] = motor_status[i].velocity;
    }
  }

  mg_jsonString = "{";
  for (int i = 0; i < MAX_MGID; i++) {
    mg_jsonString += "\"mg_";
    mg_jsonString += String(MOT_CAN_ID[i]);
    mg_jsonString += ":[";
    for (int j = 0; j < 4; j++) {
      mg_jsonString += String(mg_array_msg[i][j]);
      if (i < (mg_array_msg_size + 1)) {
        mg_jsonString += ",";
      }
    }
    if (i != MAX_MGID) {
      mg_jsonString += "],";
    } else {
      mg_jsonString += "]";
    }
  }
  mg_jsonString += "]}";

  // for (char i = 0; i < MAX_CID; i++) {
  //   if (sc.FeedBack(ID[i]) != -1) {
  //     Pos[ID[i]] = sc.ReadPos(-1);
  //     Speed[ID[i]] = sc.ReadSpeed(-1);
  //     Load[ID[i]] = sc.ReadLoad(-1);
  //     Voltage[ID[i]] = sc.ReadVoltage(-1);
  //     Temper[ID[i]] = sc.ReadTemper(-1);
  //     Current[ID[i]] = sc.ReadCurrent(-1);

  //     sm_array_msg[i][0] = Pos[ID[i]];
  //     sm_array_msg[i][1] = Speed[ID[i]];
  //     sm_array_msg[i][2] = Load[ID[i]];
  //     sm_array_msg[i][3] = Voltage[ID[i]];
  //     sm_array_msg[i][4] = Temper[ID[i]];
  //     sm_array_msg[i][5] = Current[ID[i]];

  //     //      Serial.print("sm_array_msg: ");
  //     //      for (int j = 0; j < 6; j++) {
  //     //        Serial.print(sm_array_msg[j]);
  //     //        if (j < 5) {
  //     //          Serial.print(", ");
  //     //        }
  //     //      }
  //     //      Serial.println();
  //   }
  // }

  // sm_jsonString = "{";
  // for (int i = 0; i < 4; i++) {
  //   sm_jsonString += "\"sm_";
  //   sm_jsonString += String(ID[i]);
  //   sm_jsonString += ":[";
  //   for (int j = 0; j < 6; j++) {
  //     sm_jsonString += String(sm_array_msg[i][j]);
  //     if (i < 5) {
  //       sm_jsonString += ",";
  //     }
  //   }
  //   if (i != 4) {
  //     sm_jsonString += "],";
  //   } else {
  //     sm_jsonString += "]";
  //   }
  // }
  // sm_jsonString += "]}";

  if (SERIAL) {
    if (driver[0].process_can_packet()) {
      motor_status[0] = driver[0].motor_state();
      // Serial.println("effort= " + String(motor_status[0].effort) + "  position=" + String(motor_status[0].position) + "  tempareture= " + String(motor_status[0].tempareture) + "  velocity= " + String(motor_status[0].velocity));
      Serial.println(mg_jsonString);
    }
  } else {
    mg_jsonString.toCharArray(char_mg_msg, sizeof(char_mg_msg));
    // String("effort= " + String(motor_status.effort) + "  position=" + String(motor_status.position) + "  tempareture= " + String(motor_status.tempareture) + "  velocity= " + String(motor_status.velocity)).toCharArray(speed_msg, sizeof(speed_msg));
    // String("hello").toCharArray(speed_msg, sizeof(speed_msg));
    mg_msg.data = char_mg_msg;
    // mg_msg_pub.publish(&mg_msg);

    // sm_jsonString.toCharArray(char_sm_msg, sizeof(char_sm_msg));
    // sm_msg.data = char_sm_msg;
    // sm_msg_pub.publish(&sm_msg);

    nh.spinOnce();
  }

  delay(1000);
}
