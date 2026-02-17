#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <SPI.h>
//#include <M5Stack.h>/
#include "lkm_driver_defs.hh"
#include "lkm_driver.hh"
// #include <SCServo.h>
#include <lx16a-servo.h>
#include <ArduinoJson.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
// #include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float64.h>
// #include <std_msgs/msg/int32.h>

geometry_msgs__msg__Twist msg;
std_msgs__msg__String mg_msg;
std_msgs__msg__String sm_msg;
std_msgs__msg__Float64 fb_msg;
// sensor_msgs__msg__Joy joy_msg;
// std_msgs__msg__Int32 cmd_motor_msg;

void pub_mg_msg(void *);
void pub_sm_msg(void *);
float rpmtoradps(float *);
float radpstorpm(float *);
float postosevo(float *);
void drivemotormg(float *, bool *, bool *, bool *, bool *);

rcl_publisher_t pub1, pub2, pub3;
rcl_subscription_t sub1;
// rcl_subscription_t sub2;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

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

char char_mg_msg[200];
char char_sm_msg[200];

// init MCP_CAN object
#define CAN0_INT 21  // Set INT to pin 4
MCP_CAN CAN0(5);     // Set CS to pin 9

uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID[] = {0x01, 0x05, 0x03, 0x04};
String mg_jsonString;

lkm_m5::Driver driver_1 = lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[0], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT);
lkm_m5::Driver driver_2 = lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[1], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT);
// lkm_m5::Driver driver_3 = lkm_m5::Driver(MASTER_CAN_ID, 0x03, MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT);
// lkm_m5::Driver driver_4 = lkm_m5::Driver(MASTER_CAN_ID, 0x04, MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT);
lkm_m5::MotorState motor_status[4];

uint8_t mode = MODE_SPEED;       //!< current mode
float target_pos = 90.0;         //!< motor target position
float target_vel = 0.0;          //!< motor target velocity
float target_torque = 0.0;       //!< motor target torque
float dir = 1.0f;                //!< direction for motion mode
float default_kp = 50.0f;        //!< default kp for motion mode
float default_kd = 1.0f;         //!< default kd for motion mode
float init_speed = 30.0f;        //!< initial speed
float slow_speed = 1.0f;         //!< slow speed
bool state_change_flag = false;  //!< state change flag

bool forword = true;
uint32_t speeds = 0;

float side_b1 = 0.0;
float side_b2 = 0.0;
float currentSpeed = 0.0;  // m/s
float linear_x = 0.0;
float angular_z = 0.0;
float angle_angular_z = 0.0;
float insideAngle = 0.0;
float outsideAngle = 0.0;
float insideAngleOutput = 0.0;
float outsideAngleOutput = 0.0;

#define MIN_ICR 0.45         // m
#define MAX_ICR 15.0         // m WHEEL_WIDESIDE * 6
#define WHEEL_LONGSIDE 0.54  // m
#define WHEEL_WIDESIDE 0.45  // m
#define MIN_ANGLE -90
#define MAX_ANGLE 90
#define MIN_PWM 2600
#define MAX_PWM 20000
#define RATIO_ANGLE_PWM 99.67
#define SERVO_FL_CENTER 11300
#define SERVO_FR_CENTER 11300
#define SERVO_RL_CENTER 11300
#define SERVO_RR_CENTER 11300
#define START_THREDTHOLD 0.01

LX16ABus servoBus;
LX16AServo servo_fl(&servoBus, 111);  // front left
LX16AServo servo_fr(&servoBus, 221);  // front right
LX16AServo servo_rl(&servoBus, 121);  // rear left
LX16AServo servo_rr(&servoBus, 231);  // rear right

// SCSCL sc;

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

#define LED_PIN 13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define SERIAL true
#define SERVO false
#define MOTOR true  // {"motor_1":1,"motor_2":1,"motor_3":1,"motor_4":1}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_mg_msg();
    pub_sm_msg();
  }
}

//twist message cb
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  linear_x = msg->linear.x;
  angular_z = msg->angular.z;
  // for (int c = 0; c < 4; c++) {
  //   Position_sc[c] = angular_z;
  // }
  if (abs(angular_z > 0.7)) {
    if (SERVO) {
      servo_fl.move_time_and_wait_for_sync(15000.0, 0);
      servo_fr.move_time_and_wait_for_sync(7500.0, 0);
      servo_rl.move_time_and_wait_for_sync(7500.0, 0);
      servo_rr.move_time_and_wait_for_sync(15000.0, 0);
      servoBus.move_sync_start();
    }

    if (MOTOR) {
      // drivemotormg(1, linear_x, MODE_SPEED);
      // drivemotormg(2, linear_x, MODE_SPEED);
      // drivemotormg(3, linear_x, MODE_SPEED);
      // drivemotormg(4, linear_x, MODE_SPEED);
    }
  } else if (abs(angular_z) > 0.40 && abs(angular_z) <= 0.70) {
    if (angular_z < -0.40) {
      angle_angular_z = mymap(angular_z, -0.4, -0.7, 90.0, 180.0);
    } else if (angular_z > 0.40) {
      angle_angular_z = mymap(angular_z, 0.4, 0.7, 90.0, 0.0);
    }
    servo_fl.move_time_and_wait_for_sync(angle_angular_z * RATIO_ANGLE_PWM + MIN_PWM, 0);
    servo_fr.move_time_and_wait_for_sync(angle_angular_z * RATIO_ANGLE_PWM + MIN_PWM, 0);
    servo_rl.move_time_and_wait_for_sync(angle_angular_z * RATIO_ANGLE_PWM + MIN_PWM, 0);
    servo_rr.move_time_and_wait_for_sync(angle_angular_z * RATIO_ANGLE_PWM + MIN_PWM, 0);
    servoBus.move_sync_start();

    if (MOTOR) {
      // drivemotormg(1, linear_x, MODE_SPEED);
      // drivemotormg(2, linear_x, MODE_SPEED);
      // drivemotormg(3, linear_x, MODE_SPEED);
      // drivemotormg(4, linear_x, MODE_SPEED);
    }
  } else if (abs(angular_z) <= 0.40) {
    currentSpeed = linear_x;
    side_b1 = mymap(abs(angular_z), (0.40 - START_THREDTHOLD), 0.0, MIN_ICR, MAX_ICR);
    side_b2 = mymap(abs(angular_z), (0.40 - START_THREDTHOLD), 0.0, MIN_ICR + WHEEL_WIDESIDE, MAX_ICR + WHEEL_WIDESIDE);
    insideAngle = atan((WHEEL_LONGSIDE / 2.0) / (side_b1));
    insideAngle = insideAngle * (180.0 / PI);
    outsideAngle = atan((WHEEL_LONGSIDE / 2.0) / (side_b2));
    outsideAngle = outsideAngle * (180.0 / PI);

    insideAngle = insideAngle * RATIO_ANGLE_PWM;
    outsideAngle = outsideAngle * RATIO_ANGLE_PWM;

    if (abs(angular_z) <= START_THREDTHOLD) {
      insideAngleOutput = 0.0;
      outsideAngleOutput = 0.0;
    } else if (angular_z < -START_THREDTHOLD) {
      insideAngleOutput = outsideAngle * -1.0;
      outsideAngleOutput = insideAngle * -1.0;
    } else if (angular_z > START_THREDTHOLD) {
      insideAngleOutput = insideAngle;
      outsideAngleOutput = outsideAngle;
    }

    if (insideAngleOutput > (RATIO_ANGLE_PWM * 90)) {
      insideAngleOutput = (RATIO_ANGLE_PWM * 90);
    } else if (insideAngleOutput < -(RATIO_ANGLE_PWM * 90)) {
      insideAngleOutput = -(RATIO_ANGLE_PWM * 90);
    }

    if (outsideAngleOutput > (RATIO_ANGLE_PWM * 90)) {
      outsideAngleOutput = (RATIO_ANGLE_PWM * 90);
    } else if (outsideAngleOutput < -(RATIO_ANGLE_PWM * 90)) {
      outsideAngleOutput = -(RATIO_ANGLE_PWM * 90);
    }
    servo_fl.move_time_and_wait_for_sync(SERVO_FL_CENTER - insideAngleOutput, 0);
    servo_fr.move_time_and_wait_for_sync(SERVO_FR_CENTER - outsideAngleOutput, 0);
    servo_rl.move_time_and_wait_for_sync(SERVO_RL_CENTER + insideAngleOutput, 0);
    servo_rr.move_time_and_wait_for_sync(SERVO_RR_CENTER + outsideAngleOutput, 0);
    servoBus.move_sync_start();

    if (MOTOR) {
      // drivemotormg(1, linear_x, MODE_SPEED);
      // drivemotormg(2, linear_x, MODE_SPEED);
      // drivemotormg(3, linear_x, MODE_SPEED);
      // drivemotormg(4, linear_x, MODE_SPEED);
    }
  }
  fb_msg.data = angle_angular_z * RATIO_ANGLE_PWM + MIN_PWM;
  // fb_msg.data = SERVO_FR_CENTER + outsideAngleOutput;
  RCSOFTCHECK(rcl_publish(&pub3, &fb_msg, NULL));

  // Position_sc[1] = mymap(msg->angular.z, 1.0, -1.0, 20.0, 1003.0);
  // Position_sc[2] = mymap(msg->angular.z, 1.0, -1.0, 20.0, 1003.0);
  // Position_sc[3] = mymap(msg->angular.z, 1.0, -1.0, 20.0, 1003.0);
  // // sc.SyncWritePos(ID, 4, Position_sc, 0, Speeds);
  // sc.WritePos(17, Position_sc[0], 0, Speeds[0]);
  // Serial2.println(String(msg->angular.z));
  // target_vel = INC_VELOCITY * mymap(cmd_vel_msg_data.linear.y, 1.0, -1.0, 1.0, 100.0);
}

// void joy_callback(const void *msgin) {
//   const sensor_msgs__msg__Joy *joy_msg = (const sensor_msgs__msg__Joy *)msgin;
//   // if (cmd_motor_msg->data == 1) {
//   //   for (char d = 0; d < MAX_MGID; d++) {
//   //     driver[d].motor_on();
//   //   }
//   // }
//   // if (cmd_motor_msg->data == 0) {
//   //   for (char d = 0; d < MAX_MGID; d++) {
//   //     driver[d].motor_off();
//   //   }
//   // }
// }

void setup() {
  if (SERIAL) {
    Serial.begin(115200);
    // Serial.print("MOSI: ");
    // Serial.println(MOSI);
    // Serial.print("MISO: ");
    // Serial.println(MISO);
    // Serial.print("SCK: ");
    // Serial.println(SCK);
    // Serial.print("SS: ");
    // Serial.println(SS);
  } else {
    set_microros_transports();
    // pinMode(LED_PIN, OUTPUT);
    // digitalWrite(LED_PIN, HIGH);

    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node_0", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
      &sub1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "my_cmd_vel"));

    // RCCHECK(rclc_subscription_init_default(
    //   &sub2,
    //   &node,
    //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    //   "my_joy"));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
      &pub1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "mg_msg/raw"));

    RCCHECK(rclc_publisher_init_default(
      &pub2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "sm_msg/raw"));

    RCCHECK(rclc_publisher_init_default(
      &pub3,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "feedback/raw"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub1, &msg, &cmd_vel_callback, ALWAYS));
    // RCCHECK(rclc_executor_add_subscription(&executor, &sub2, &joy_msg, &joy_callback, ALWAYS));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    std_msgs__msg__String__init(&mg_msg);
    std_msgs__msg__String__init(&sm_msg);
  }

  // pinMode(5, OUTPUT);

  if (MOTOR) {
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
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

    pinMode(CAN0_INT, INPUT);

    driver_1.init(&CAN0);
    driver_2.init(&CAN0);
    // driver_3.init(&CAN0);
    // driver_4.init(&CAN0);

    driver_1.motor_on();
    driver_2.motor_on();
    // driver_3.motor_on();
    // driver_4.motor_on();

    // for (char a = 0; a < MAX_MGID; a++) {
    //   driver[a].init(&CAN0);
    //   driver[a].motor_on();
    // }
  }

  if (SERVO) {
    // Serial2.begin(115200, SERIAL_8N1, 16, 17);
    Serial2.begin(1000000, SERIAL_8N1, 16, 17);
    // sc.pSerial = &Serial2;

    servoBus.begin(&Serial2, 16);  // on TX pin 16  // use pin 2 as the TX flag for buffer
    servoBus.retry = 1;            // enforce synchronous real time
    servoBus.debug(true);
  }
}


float millis_pre = -60000.0, millis_set = 5000.0;
int pos = 2600;
JsonDocument doc;

void loop() {
  if (SERIAL) {
    mode = MODE_SPEED;
    // sc.WritePos(17, pos, 0, Speeds[0]);
    if (MOTOR) {
      static float SpeedforTest[] = { 0.0, 0.0, 0.0, 0.0 };
      // target_vel = rpmtoradps(SpeedforTest);
      if (millis() - millis_pre >= 5000) { // 60000
        driver_1.stop_motor();
        driver_2.stop_motor();
        // driver_3.stop_motor();
        // driver_4.stop_motor();
        Serial.println("Enter_motor_mg_speed_Json!");
        if (Serial.available() > 0) {
          DeserializationError error = deserializeJson(doc, Serial.readStringUntil('\n'));  //    {"motor_1":1,"motor_2":1,"motor_3":1,"motor_4":1}
          if (error) {
            Serial.println("ERROR DeserialzationError!");
            Serial.println(error.c_str());
          } else {
            SpeedforTest[0] = doc["motor_1_speed"];
            SpeedforTest[1] = doc["motor_2_speed"];
            SpeedforTest[2] = doc["motor_2_position"];
            millis_pre = millis();
          }//    {"motor_1_speed":10,"motor_2_speed":20,"motor_2_position":30}
        }
      } else {
        pub_mg_msg();
        // driver_1.single_loop_angle_with_speed_control(0x00, SpeedforTest[2], rpmtoradps(SpeedforTest[1]));
        driver_1.speed_closed_loop_control(rpmtoradps(SpeedforTest[0]));
        if(SpeedforTest[2] > 18.0){
          SpeedforTest[2] = 18.0;
          driver_2.single_loop_angle_with_speed_control(0x00, SpeedforTest[2], rpmtoradps(SpeedforTest[1]));
          SpeedforTest[3] = SpeedforTest[2];
        }else if (SpeedforTest[2] < SpeedforTest[3]){
          driver_2.single_loop_angle_with_speed_control(0x01, SpeedforTest[2], rpmtoradps(SpeedforTest[1]));
          SpeedforTest[3] = SpeedforTest[2];
        }else{
          driver_2.single_loop_angle_with_speed_control(0x00, SpeedforTest[2], rpmtoradps(SpeedforTest[1]));
          SpeedforTest[3] = SpeedforTest[2];
        }
        // driver_2.increment_angle_with_speed_control(SpeedforTest[2], rpmtoradps(SpeedforTest[1]));
        // driver_2.speed_closed_loop_control(rpmtoradps(SpeedforTest[1]));
        // driver_3.speed_closed_loop_control(rpmtoradps(SpeedforTest[2]));
        // driver_4.speed_closed_loop_control(rpmtoradps(SpeedforTest[3]));
        // driver_1.speed_closed_loop_control(rpmtoradps(1));
        // driver_2.speed_closed_loop_control(rpmtoradps(1));
        // driver_3.speed_closed_loop_control(rpmtoradps(1));
        // driver_4.speed_closed_loop_control(rpmtoradps(1));
      }
    }

    if (SERVO) {
      if (millis() - millis_pre >= millis_set) {
        millis_pre = millis();

        servo_fl.move_time_and_wait_for_sync(pos, 0);
        servoBus.move_sync_start();

        if (pos < 20000) {
          pos += 10000;
          if (pos > 20000) {
            pos = 20000;
          }
        } else {
          pos = 2600;
        }
      }
    }
    delay(1000);
  } else {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(600)));
  }
  // String("effort= " + String(motor_status.effort) + "  position=" + String(motor_status.position) + "  tempareture= " + String(motor_status.tempareture) + "  velocity= " + String(motor_status.velocity)).toCharArray(speed_msg, sizeof(speed_msg));
}

void pub_mg_msg() {
  if (MOTOR) {
    for (char b = 0; b < MAX_MGID; b++) {
      switch (b + 1) {
        case 2:
          if (driver_2.process_can_packet()) {
            motor_status[b] = driver_2.motor_state();
            mg_array_msg[b][0] = motor_status[b].effort;
            mg_array_msg[b][1] = motor_status[b].position;
            mg_array_msg[b][2] = motor_status[b].tempareture;
            mg_array_msg[b][3] = radpstorpm(motor_status[b].velocity);
          }
          break;
        // case 3:
        //   if (driver_3.process_can_packet()) {
        //     motor_status[b] = driver_3.motor_state();
        //     mg_array_msg[b][0] = motor_status[b].effort;
        //     mg_array_msg[b][1] = motor_status[b].position;
        //     mg_array_msg[b][2] = motor_status[b].tempareture;
        //     mg_array_msg[b][3] = radpstorpm(motor_status[b].velocity);
        //   }
        //   break;
        // case 4:
        //   if (driver_4.process_can_packet()) {
        //     motor_status[b] = driver_4.motor_state();
        //     mg_array_msg[b][0] = motor_status[b].effort;
        //     mg_array_msg[b][1] = motor_status[b].position;
        //     mg_array_msg[b][2] = motor_status[b].tempareture;
        //     mg_array_msg[b][3] = radpstorpm(motor_status[b].velocity);
        //   }
        //   break;
        default:
          if (driver_1.process_can_packet()) {
            motor_status[b] = driver_1.motor_state();
            mg_array_msg[b][0] = motor_status[b].effort;
            mg_array_msg[b][1] = motor_status[b].position;
            mg_array_msg[b][2] = motor_status[b].tempareture;
            mg_array_msg[b][3] = radpstorpm(motor_status[b].velocity);
          }
      }
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

  if (SERIAL) {
    Serial.println(mg_jsonString);
  } else {
    mg_jsonString.toCharArray(char_mg_msg, sizeof(char_mg_msg));
    rosidl_runtime_c__String__assign(&mg_msg.data, char_mg_msg);
    RCSOFTCHECK(rcl_publish(&pub1, &mg_msg, NULL));
  }
}

void pub_sm_msg() {
  if (SERVO) {
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
  }

  sm_jsonString = "{";
  for (int i = 0; i < 4; i++) {
    sm_jsonString += "\"sm_";
    sm_jsonString += String(ID[i]);
    sm_jsonString += ":[";
    for (int j = 0; j < 6; j++) {
      sm_jsonString += String(sm_array_msg[i][j]);
      if (i < 5) {
        sm_jsonString += ",";
      }
    }
    if (i != 4) {
      sm_jsonString += "],";
    } else {
      sm_jsonString += "]";
    }
  }
  sm_jsonString += "]}";

  if (SERIAL) {
    Serial.println(sm_jsonString);
  } else {
    sm_jsonString.toCharArray(char_sm_msg, sizeof(char_sm_msg));
    rosidl_runtime_c__String__assign(&sm_msg.data, char_sm_msg);
    RCSOFTCHECK(rcl_publish(&pub2, &sm_msg, NULL));
  }
}

float rpmtoradps(float rpm) {
  return ((rpm * 2 * PI) / 60) * 6;
}

float radpstorpm(float radps) {
  return (radps / 6) * (30 / PI);
}

float postosevo(float pos) {
  return mymap(pos, MIN_ANGLE, MAX_ANGLE, MAX_PWM, MIN_PWM);
}

void drivemotormg(char motor_id, float speeds_mps, uint8_t Mode) {
  static bool speeds_cmd = false;
  if (abs(speeds_mps) <= START_THREDTHOLD) {
    speeds_cmd = false;
    driver_1.stop_motor();
    // driver_2.stop_motor();
    // driver_3.stop_motor();
    // driver_4.stop_motor();
  } else if (speeds_mps > START_THREDTHOLD) {
    speeds_cmd = true;
  } else if (speeds_mps < -START_THREDTHOLD) {
    speeds_cmd = true;
  }

  if (speeds_cmd) {
    // if (Mode == MODE_POSITION) {
    //   // set limit speed when state changed
    //   switch (motor_id) {
    //     case 2:
    //       driver_2.multi_loop_angle_with_speed_control(target_pos, slow_speed);
    //       break;
    //     case 3:
    //       driver_3.multi_loop_angle_with_speed_control(target_pos, slow_speed);
    //       break;
    //     case 4:
    //       driver_4.multi_loop_angle_with_speed_control(target_pos, slow_speed);
    //       break;
    //     default:
    //       driver_1.multi_loop_angle_with_speed_control(target_pos, slow_speed);
    //   }
    // } else if (Mode == MODE_SPEED) {
    //   switch (motor_id) {
    //     case 2:
    //       driver_2.speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
    //       break;
    //     case 3:
    //       driver_3.speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
    //       break;
    //     case 4:
    //       driver_4.speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
    //       break;
    //     default:
    //       driver_1.speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
    //   }
    // } else if (Mode == MODE_CURRENT) {
    //   if (driver_1.motor_type() != MOTOR_SERIES_MS && driver_2.motor_type() != MOTOR_SERIES_MS && driver_3.motor_type() != MOTOR_SERIES_MS && driver_4.motor_type() != MOTOR_SERIES_MS) {
    //     switch (motor_id) {
    //       case 2:
    //         driver_2.torque_closed_loop_control(target_torque);
    //         break;
    //       case 3:
    //         driver_3.torque_closed_loop_control(target_torque);
    //         break;
    //       case 4:
    //         driver_4.torque_closed_loop_control(target_torque);
    //         break;
    //       default:
    //         driver_1.torque_closed_loop_control(target_torque);
    //     }
    //   } else {
    //     switch (motor_id) {
    //       case 2:
    //         driver_2.open_loop_control(target_torque * 100);
    //         break;
    //       case 3:
    //         driver_3.open_loop_control(target_torque * 100);
    //         break;
    //       case 4:
    //         driver_4.open_loop_control(target_torque * 100);
    //         break;
    //       default:
    //         driver_1.open_loop_control(target_torque * 100);
    //     }
    //   }
    // }
  }
}