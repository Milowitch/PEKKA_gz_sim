// MG_Motor
#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
//#include <M5Stack.h>/
#include "lkm_driver_defs.hh"
#include "lkm_driver.hh"
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// Micro_ROS
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float64.h>
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

// MG_Motor
float mymap(float, float, float, float, float);
float rpmtoradps(float);
float radpstorpm(float);
void pub_mg_msg();
void drivemotor_mg_speed(uint8_t, float);
void drivemotor_steering_arkerman(uint8_t, uint8_t, uint8_t, uint8_t, float, float, float);
void drivemotor_steering(uint8_t, float, float);
void drivemotor_mg_sync_speed(float);

#define INC_POSITION 20.0
#define INC_VELOCITY 1.0
#define INC_TORQUE 0.04
#define MODE_POSITION 0x01
#define MODE_SPEED 0x02
#define MODE_CURRENT 0x03.

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

// init MCP_CAN object
// #define CAN0_INT 21  // Set INT to pin 4
MCP_CAN CAN0(9);  // Set CS to pin 9

#define MAX_MGID 8
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

lkm_m5::Driver driver_test = lkm_m5::Driver(MASTER_CAN_ID, 0x03, MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT);
lkm_m5::MotorState motor_status_test;
// lkm_m5::Driver driver[8] = {
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[0], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[1], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[2], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[3], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[4], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[5], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[6], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
//   lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[7], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT)
// };
lkm_m5::MotorState motor_status[8];
float mg_array_msg[8][4];
int mg_array_msg_size = sizeof(mg_array_msg) / sizeof(mg_array_msg[0]);

float mymap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// Micro_ROS
rcl_publisher_t pub1;
// rcl_publisher_t pub2;
rcl_publisher_t pub3;
rcl_subscription_t sub1;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool micro_ros_init_successful;

std_msgs__msg__Float64MultiArray msg;
std_msgs__msg__String mg_msg;
// std_msgs__msg__String sm_msg;
std_msgs__msg__Float64 fb_msg;

#define LED_PIN 13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // pub_mg_msg();
    // RCSOFTCHECK(rcl_publish(&pub1, &msg, NULL));
  }
}

//message cb
void ros_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  static double cmd_msg[3];
  for (char i = 0; i < msg->data.capacity; i++) {
    cmd_msg[i] = msg->data.data[i];
  }
  // drivemotor_mg_speed(0x01, cmd_msg[0]);
  // drivemotor_mg_speed(0x02, cmd_msg[0]);
  // drivemotor_mg_speed(0x03, cmd_msg[0]);
  // drivemotor_mg_speed(0x04, cmd_msg[0]);
  driver_test.speed_closed_loop_control(rpmtoradps(mymap(cmd_msg[0], -1.0, 1.0, -300, 300)));

  fb_msg.data = rpmtoradps(mymap(cmd_msg[0], -1.0, 1.0, -300, 300));
  RCSOFTCHECK(rcl_publish(&pub3, &fb_msg, NULL));

  if (cmd_msg[2] == 1.0) {
    // for (int i = 0; i < MAX_MGID; i++) {
    //   driver[i].clear_motor_error_state();
    //   driver[i].motor_off();
    //   driver[i].motor_on();
    // }
    driver_test.clear_motor_error_state();
    driver_test.motor_off();
    driver_test.motor_on();
  }
  // drivemotor_steering(0x05, cmd_msg[1], 20.0);
  // if (abs(cmd_msg[1] > 0.7)) {
  // } else if (abs(cmd_msg[1]) > 0.40 && abs(cmd_msg[1]) <= 0.70) {
  // } else if (abs(cmd_msg[1]) <= 0.40) {
  //   // drivemotor_steering_arkerman(0x01, 0x02, 0x03, 0x04, cmd_msg[1], 30, cmd_msg[0]);
  // }
}
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
void setup() {
  // MG_Motor
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    // Serial.println("MCP2515 Initialized Successfully!");
    // nh.loginfo("MCP2515 Initialized Successfully!");
  } else {
    // Serial.println("Error Initializing MCP2515...");
    // nh.loginfo("Error Initializing MCP2515...");
  }
  CAN0.setMode(MCP_NORMAL);

  // for (int i = 0; i < MAX_MGID; i++) {
  //   driver[i].init(&CAN0);
  //   driver[i].motor_on();
  // }
  driver_test.init(&CAN0);
  driver_test.motor_on();
  //^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

  // Micro_ROS
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node_drive_system", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &pub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "mg_msg/raw"));

  RCCHECK(rclc_publisher_init_default(
    &pub3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "feedback/raw"));

  // create timer,
  timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &sub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "my_cmd"));

  // Init the memory of your array in order to provide it to the executor.
  // If a message from ROS comes and it is bigger than this, it will be ignored, so ensure that capacities here are big enought.

  msg.data.capacity = 3;
  msg.data.size = 0;
  msg.data.data = (double *)malloc(msg.data.capacity * sizeof(double));

  msg.layout.dim.capacity = 3;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < msg.layout.dim.capacity; i++) {
    msg.layout.dim.data[i].label.capacity = 5;
    msg.layout.dim.data[i].label.size = 0;
    msg.layout.dim.data[i].label.data = (char *)malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // ENSURE HERE THAT ALL THE MEMBERS ARE INIT RECURSIVELY, I HAVE JUST COPIED YOUR CODE

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // unsigned int rcl_wait_timeout = 1000;  // in ms
  // RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub1, &msg, &ros_cmd_callback, ALWAYS));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  std_msgs__msg__String__init(&mg_msg);
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500)));
}

// funtion
float rpmtoradps(float rpm) {
  return ((rpm * 2 * PI) / 60) * 6;
}

float radpstorpm(float radps) {
  return (radps / 6) * (30 / PI);
}

// void drivemotor_mg_speed(uint8_t motor_id, float speeds_mps) {
//   static bool speeds_cmd = false;
//   static int8_t index_id = -1;
//   for (int i = 0; i < sizeof(MOT_CAN_ID) / sizeof(MOT_CAN_ID[0]); i++) {
//     if (MOT_CAN_ID[i] == motor_id) {
//       index_id = i;
//       break;
//     }
//   }

//   if (index_id != -1) {
//     if (abs(speeds_mps) <= START_THREDTHOLD) {
//       speeds_cmd = false;
//       driver[index_id].stop_motor();
//     } else if (speeds_mps > START_THREDTHOLD || speeds_mps < -START_THREDTHOLD) {
//       speeds_cmd = true;
//     }

//     if (speeds_cmd) {
//       driver[index_id].speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
//     }
//   }
// }

// float check_angle(float angle_in) {
//   if (angle_in > 18.0) {
//     return 18.0;
//   } else if (angle_in < 0.0) {
//     return 0.0;
//   } else {
//     return angle_in;
//   }
// }

// void drivemotor_steering_arkerman(uint8_t motor_id_fl, uint8_t motor_id_fr, uint8_t motor_id_rl, uint8_t motor_id_rr, float angular, float angular_speeds_rpm, float speed_mps) {
//   static float side_b1 = 0.0;
//   static float side_b2 = 0.0;
//   static float insideAngle = 0.0;
//   static float outsideAngle = 0.0;
//   static float insideAngleOutput = 0.0;
//   static float outsideAngleOutput = 0.0;
//   static float angle_angular_fl = 0.0, angle_angular_fl_pre = 0.0;
//   static float angle_angular_fr = 0.0, angle_angular_fr_pre = 0.0;
//   static float angle_angular_rl = 0.0, angle_angular_rl_pre = 0.0;
//   static float angle_angular_rr = 0.0, angle_angular_rr_pre = 0.0;
//   static float insideVel = 0.0;
//   static float outsideVel = 0.0;

//   side_b1 = mymap(abs(angular), (0.40 - START_THREDTHOLD), 0.0, MIN_ICR, MAX_ICR);
//   side_b2 = mymap(abs(angular), (0.40 - START_THREDTHOLD), 0.0, MIN_ICR + WHEEL_WIDESIDE, MAX_ICR + WHEEL_WIDESIDE);
//   insideAngle = atan((WHEEL_LONGSIDE / 2.0) / (side_b1));
//   insideAngle = insideAngle * (180.0 / PI);
//   outsideAngle = atan((WHEEL_LONGSIDE / 2.0) / (side_b2));
//   outsideAngle = outsideAngle * (180.0 / PI);

//   insideAngle = insideAngle / 10;
//   outsideAngle = outsideAngle / 10;

//   if (abs(angular) <= START_THREDTHOLD) {
//     insideAngleOutput = 0.0;
//     outsideAngleOutput = 0.0;
//   } else if (angular < -START_THREDTHOLD) {
//     insideAngleOutput = outsideAngle * -1.0;
//     outsideAngleOutput = insideAngle * -1.0;
//   } else if (angular > START_THREDTHOLD) {
//     insideAngleOutput = insideAngle;
//     outsideAngleOutput = outsideAngle;
//   }

//   if (insideAngleOutput > 9.0) {
//     insideAngleOutput = 9.0;
//   } else if (insideAngleOutput < -9.0) {
//     insideAngleOutput = -9.0;
//   }

//   if (outsideAngleOutput > 9.0) {
//     outsideAngleOutput = 9.0;
//   } else if (outsideAngleOutput < -9.0) {
//     outsideAngleOutput = -9.0;
//   }

//   angle_angular_fl = constrain(9.0 + insideAngleOutput, 0.0, 18.0);
//   angle_angular_fr = constrain(9.0 + outsideAngleOutput, 0.0, 18.0);

//   insideVel = float(rpmtoradps(mymap(speed_mps, -1.0, 1.0, -300, 300)) * outsideAngle);
//   outsideVel = float(rpmtoradps(mymap(speed_mps, -1.0, 1.0, -300, 300)) * insideAngle);
//   insideVel = constrain(insideVel, -rpmtoradps(300), rpmtoradps(300));
//   outsideVel = constrain(outsideVel, -rpmtoradps(300), rpmtoradps(300));

//   if (angle_angular_fl < angle_angular_fl_pre) {
//     driver[motor_id_fl].single_loop_angle_with_speed_control(0x01, angle_angular_fl, rpmtoradps(angular_speeds_rpm));
//     angle_angular_fl_pre = angle_angular_fl;
//   } else {
//     driver[motor_id_fl].single_loop_angle_with_speed_control(0x00, angle_angular_fl, rpmtoradps(angular_speeds_rpm));
//     angle_angular_fl_pre = angle_angular_fl;
//   }

//   if (angle_angular_fr < angle_angular_fr_pre) {
//     driver[motor_id_fr].single_loop_angle_with_speed_control(0x01, angle_angular_fr, rpmtoradps(angular_speeds_rpm));
//     angle_angular_fr_pre = angle_angular_fr;
//   } else {
//     driver[motor_id_fr].single_loop_angle_with_speed_control(0x00, angle_angular_fr, rpmtoradps(angular_speeds_rpm));
//     angle_angular_fr_pre = angle_angular_fr;
//   }

//   drivemotor_mg_speed(motor_id_fl, insideVel);
//   drivemotor_mg_speed(motor_id_fr, outsideVel);
//   drivemotor_mg_speed(motor_id_rl, insideVel);
//   drivemotor_mg_speed(motor_id_rr, outsideVel);
// }

// void drivemotor_steering(uint8_t motor_id, float angular, float angular_speeds_rpm) {
//   static float angle_angular_z = 0.0;
//   static float angle_angular_z_pre = 0.0;
//   static int8_t index_id = -1;

//   for (int i = 0; i < sizeof(MOT_CAN_ID) / sizeof(MOT_CAN_ID[0]); i++) {
//     if (MOT_CAN_ID[i] == motor_id) {
//       index_id = i;
//       break;
//     }
//   }

//   if (angular < -0.40) {
//     angle_angular_z = mymap(angular, -0.4, -0.7, 9.0, 18.0);
//   } else if (angular > 0.40) {
//     angle_angular_z = mymap(angular, 0.4, 0.7, 9.0, 0.0);
//   } else {
//     angle_angular_z = 9.0;
//   }

//   if (angle_angular_z > 18.0) {
//     angle_angular_z = 18.0;
//   } else if (angle_angular_z < 0.0) {
//     angle_angular_z = 0.0;
//   }

//   if (angle_angular_z < angle_angular_z_pre) {
//     driver[index_id].single_loop_angle_with_speed_control(0x01, angle_angular_z, rpmtoradps(angular_speeds_rpm));
//     angle_angular_z_pre = angle_angular_z;
//   } else {
//     driver[index_id].single_loop_angle_with_speed_control(0x00, angle_angular_z, rpmtoradps(angular_speeds_rpm));
//     angle_angular_z_pre = angle_angular_z;
//   }
// }

// void pub_mg_msg() {
//   static String mg_jsonString;
//   static char char_mg_msg[300];

//   mg_jsonString = "{";
//   for (int i = 0; i < MAX_MGID; i++) {
//     if (driver[i].process_can_packet()) {
//       motor_status[i] = driver[i].motor_state();
//       mg_array_msg[i][0] = motor_status[i].effort;
//       mg_array_msg[i][1] = motor_status[i].position;
//       mg_array_msg[i][2] = motor_status[i].tempareture;
//       mg_array_msg[i][3] = radpstorpm(motor_status[i].velocity);
//     }

//     mg_jsonString += "\"mg_";
//     mg_jsonString += String(MOT_CAN_ID[i]);
//     mg_jsonString += ":[";
//     for (int j = 0; j < mg_array_msg_size; j++) {
//       mg_jsonString += String(mg_array_msg[i][j]);
//       if (j < (mg_array_msg_size - 1)) {
//         mg_jsonString += ",";
//       }
//     }
//     if (i < (MAX_MGID - 1)) {
//       mg_jsonString += "],";
//     }
//   }
//   mg_jsonString += "]}";

//   mg_jsonString.toCharArray(char_mg_msg, sizeof(char_mg_msg));
//   rosidl_runtime_c__String__assign(&mg_msg.data, char_mg_msg);
//   RCSOFTCHECK(rcl_publish(&pub1, &mg_msg, NULL));
// }

// void drivemotor_mg_sync_speed(float speeds_mps) {
//   static bool speeds_cmd = false;

//   if (abs(speeds_mps) <= START_THREDTHOLD) {
//     speeds_cmd = false;
//     for (int i = 0; i < MAX_MGID; i++) {
//       driver[i].stop_motor();
//     }
//   } else if (speeds_mps > START_THREDTHOLD || speeds_mps < -START_THREDTHOLD) {
//     speeds_cmd = true;
//   }

//   if (speeds_cmd) {
//     driver[0].speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
//     if (mg_array_msg[0][3] != 0.0) {
//       driver[1].speed_closed_loop_control(rpmtoradps(mg_array_msg[0][3]));
//     } else {
//       driver[1].speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
//     }
//     driver[2].speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
//     if (mg_array_msg[2][3] != 0.0) {
//       driver[3].speed_closed_loop_control(rpmtoradps(mg_array_msg[2][3]));
//     } else {
//       driver[3].speed_closed_loop_control(rpmtoradps(mymap(speeds_mps, -1.0, 1.0, -300, 300)));
//     }
//   }
// }
//^^^ funtion ^^^^^^^^^