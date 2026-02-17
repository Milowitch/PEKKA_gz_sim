// MG_Motor
#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
//#include <M5Stack.h>/
#include "lkm_driver_defs.hh"
#include "lkm_driver.hh"
#include <ArduinoJson.h>
#include <FlashStorage.h>
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// Micro_ROS
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float64.h>
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

float mymap(float, float, float, float, float);
float rpmtoradps(float);
float radpstorpm(float);
void drivemotor_steering(float, unsigned long, float);
void drivemotor_steering_arkerman(float, float, float);

// MG_Motor
#define INC_POSITION 20.0
#define INC_VELOCITY 1.0
#define INC_TORQUE 0.04
#define MODE_POSITION 0x01
#define MODE_SPEED 0x02
#define MODE_CURRENT 0x03
// init MCP_CAN object
MCP_CAN CAN0(9);
MCP_CAN CAN1(10);

#define CAN0_INT 2
#define CAN1_INT 3

#define MAX_MGID 4
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID[MAX_MGID] = { 0x01, 0x02, 0x03, 0x04 };
lkm_m5::Driver driver[MAX_MGID] = {
  lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[0], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[1], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[2], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[3], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  // lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[4], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
  // lkm_m5::Driver(MASTER_CAN_ID, MOT_CAN_ID[5], MOTOR_SERIES_MG, ENCODER_TYPE_16_BIT),
};
lkm_m5::MotorState motor_status[MAX_MGID];

#define MAX_MGID_1 2
uint8_t MASTER_CAN_ID_1 = 0x00;
uint8_t MOT_CAN_ID_1[MAX_MGID_1] = { 0x05, 0x06};
lkm_m5::Driver driver_1[MAX_MGID_1] = {
  lkm_m5::Driver(MASTER_CAN_ID_1, MOT_CAN_ID_1[0], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  lkm_m5::Driver(MASTER_CAN_ID_1, MOT_CAN_ID_1[1], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  // lkm_m5::Driver(MASTER_CAN_ID_1, MOT_CAN_ID_1[2], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
  // lkm_m5::Driver(MASTER_CAN_ID_1, MOT_CAN_ID_1[3], MOTOR_SERIES_MG, ENCODER_TYPE_14_BIT),
};
lkm_m5::MotorState motor_status_1[MAX_MGID_1];

#define MIN_ICR 0.45          // m
#define MAX_ICR 2.70          // m WHEEL_WIDESIDE * 6
#define WHEEL_LONGSIDE 0.700  // m
#define WHEEL_WIDESIDE 0.585  // m
#define MIN_ANGLE -90
#define MAX_ANGLE 90
#define MIN_PWM 0.0
#define MAX_PWM 18.0
#define SERVO_FL_CENTER 9.0
#define SERVO_FR_CENTER 9.0
#define SERVO_RL_CENTER 9.0
#define SERVO_RR_CENTER 9.0
#define START_THREDTHOLD 0.01

struct MyData {
  float position[4];
};

FlashStorage(my_flash_store, MyData);
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
std_msgs__msg__String motor_msg;
// std_msgs__msg__String st_msg;
// std_msgs__msg__Float64 fb_msg;
std_msgs__msg__String fb_msg;

#define LED_PIN 13
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    // pub_motor_msg
    const int capacity = JSON_ARRAY_SIZE(6) + 6 * JSON_OBJECT_SIZE(5);
    StaticJsonDocument<capacity> doc;
    static char char_motor_msg[capacity];

    for (int i = 0; i < MAX_MGID; i++) {
      doc[i]["name"] = "mg_" + String(MOT_CAN_ID[i]);
      if (driver[i].process_can_packet()) {
        motor_status[i] = driver[i].motor_state();
        doc[i]["effort"] = motor_status[i].effort;
        doc[i]["position"] = motor_status[i].position;
        doc[i]["tempareture"] = motor_status[i].tempareture;
        doc[i]["velocity"] = radpstorpm(motor_status[i].velocity);
      } else {
        doc[i]["effort"] = motor_status[i].effort;
        doc[i]["position"] = motor_status[i].position;
        doc[i]["tempareture"] = motor_status[i].tempareture;
        doc[i]["velocity"] = radpstorpm(motor_status[i].velocity);
      }
    }
    for (int i = 0; i < MAX_MGID_1; i++) {
      doc[i + (MAX_MGID + 1)]["name"] = "mg_" + String(MOT_CAN_ID_1[i]);
      if (driver_1[i].process_can_packet()) {
        motor_status_1[i] = driver_1[i].motor_state();
        doc[i + (MAX_MGID + 1)]["effort"] = motor_status_1[i].effort;
        doc[i + (MAX_MGID + 1)]["position"] = motor_status_1[i].position;
        doc[i + (MAX_MGID + 1)]["tempareture"] = motor_status_1[i].tempareture;
        doc[i + (MAX_MGID + 1)]["velocity"] = radpstorpm(motor_status_1[i].velocity);
      } else {
        doc[i + (MAX_MGID + 1)]["effort"] = motor_status_1[i].effort;
        doc[i + (MAX_MGID + 1)]["position"] = motor_status_1[i].position;
        doc[i + (MAX_MGID + 1)]["tempareture"] = motor_status_1[i].tempareture;
        doc[i + (MAX_MGID + 1)]["velocity"] = radpstorpm(motor_status_1[i].velocity);
      }
    }
    serializeJson(doc, char_motor_msg, sizeof(char_motor_msg));
    rosidl_runtime_c__String__assign(&motor_msg.data, char_motor_msg);
    rcl_publish(&pub1, &motor_msg, NULL);
    //^^^^^^^^^^ pub_motor_msg ^^^^^^^^^^^^^^^^^^^^^^
  }
}

MyData data;
//message cb
void ros_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  static float cmd_msg[6];
  for (int i = 0; i < msg->data.capacity; i++) {
    cmd_msg[i] = msg->data.data[i];
  }

  if (cmd_msg[3] == 1.0) {
    for (int i = 0; i < MAX_MGID; i++) {
      driver[i].clear_motor_error_state();
      driver[i].motor_on();
    }
    for (int i = 0; i < MAX_MGID_1; i++) {
      driver_1[i].clear_motor_error_state();
      driver_1[i].motor_on();
    }
  }

  if (cmd_msg[4] == 1.0) {
    for (int i = 0; i < MAX_MGID; i++) {
      driver[i].motor_off();
    }
    for (int i = 0; i < MAX_MGID_1; i++) {
      driver_1[i].motor_off();
    }
  }

  // if (abs(cmd_msg[1] > 0.7)) {
  // } else if (abs(cmd_msg[1]) > 0.40 && abs(cmd_msg[1]) <= 0.70) {
  // } else if (abs(cmd_msg[1]) <= 0.40) {
  //   // drivemotor_steering_arkerman(&driver[0], &driver[1], &driver[2], &driver[3], &driver[0], &driver[1], cmd_msg[1], 30, cmd_msg[0]);
  // }
  if (cmd_msg[2] == 0.0) {
    drivemotor_steering(cmd_msg[1], 1, cmd_msg[0]);
  } else if (cmd_msg[2] == 1.0) {
    drivemotor_steering_arkerman(cmd_msg[1], 1, cmd_msg[0]);
  } else if (cmd_msg[2] == 3.0) {
    static float increment = 2.0;
    static float angle_angular_z[4] = { 0.0, 0.0, 0.0, 0.0 };
    static float angle_angular_z_pre[4] = { 9.0, 9.0, 9.0, 9.0 };

    if (cmd_msg[5] == 5.0 || cmd_msg[5] == 6.0) {
      if (cmd_msg[5] == 5.0) {
        angle_angular_z[0] = round(mymap(cmd_msg[1], -1.0, 1.0, 0.0, 18.0));
        driver_1[0].multi_loop_angle_with_speed_control(angle_angular_z[0], 1.0);
      } else if (cmd_msg[5] == 6.0) {
        angle_angular_z[1] = round(mymap(cmd_msg[1], -1.0, 1.0, 0.0, 18.0));
        driver_1[1].multi_loop_angle_with_speed_control(angle_angular_z[1], 1.0);
      }
      // if (angle_angular_z[number] > angle_angular_z_pre[number]) {
      //   angle_angular_z_pre[number] += increment;
      // } else if (angle_angular_z[number] < angle_angular_z_pre[number]) {
      //   angle_angular_z_pre[number] -= increment;
      // }
      // driver_1[number].multi_loop_angle_with_speed_control(angle_angular_z_pre[number], 1.0);
    } else {
      static float speed_goal;
      speed_goal = rpmtoradps(mymap(cmd_msg[0], -1.0, 1.0, -300, 300));
      if (abs(speed_goal) > 0.0) {
        if (cmd_msg[5] == 1.0) {
          driver[0].speed_closed_loop_control(speed_goal);
        } else if (cmd_msg[5] == 2.0) {
          driver[1].speed_closed_loop_control(speed_goal);
        } else if (cmd_msg[5] == 3.0) {
          driver[2].speed_closed_loop_control(-speed_goal);
        } else if (cmd_msg[5] == 4.0) {
          driver[3].speed_closed_loop_control(-speed_goal);
        }
      } else {
        driver[0].stop_motor();
        driver[1].stop_motor();
        driver[2].stop_motor();
        driver[3].stop_motor();
      }
    }
  }

  const int fb_capacity = JSON_ARRAY_SIZE(6);
  StaticJsonDocument<fb_capacity> fd_doc;
  static char char_fb_msg[fb_capacity];

  // fd_doc[0] = cmd_msg[0];
  // fd_doc[1] = cmd_msg[1];
  // fd_doc[2] = cmd_msg[2];
  // fd_doc[3] = cmd_msg[3];
  // fd_doc[4] = cmd_msg[4];
  // fd_doc[5] = cmd_msg[5];

  // fd_doc[6] = angle_angular_z_pre[0];
  // fd_doc[7] = angle_angular_z_pre[1];
  for (int i = 0; i < 2; i++) {
    fd_doc[i] = motor_status_1[i].position;
  }

  serializeJson(fd_doc, char_fb_msg, sizeof(char_fb_msg));
  rosidl_runtime_c__String__assign(&fb_msg.data, char_fb_msg);
  rcl_publish(&pub3, &fb_msg, NULL);
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities() {
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
    "motor_msg/rawdata"));

  RCCHECK(rclc_publisher_init_default(
    &pub3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "feedback/rawdata"));

  // create timer,
  timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 500;
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

  msg.data.capacity = 6;
  msg.data.size = 0;
  msg.data.data = (double *)malloc(msg.data.capacity * sizeof(double));

  msg.layout.dim.capacity = 6;
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

  // rosidl_runtime_c__String__init(&wh_msg.data);

  // MG_Motor
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    // Serial.print("CAN0: Init OK!\r\n");
    CAN0.setMode(MCP_NORMAL);
  }
  // else Serial.print("CAN0: Init Fail!!!\r\n");

  if (CAN1.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    // Serial.print("CAN1: Init OK!\r\n");
    CAN1.setMode(MCP_NORMAL);
  }
  // else Serial.print("CAN1: Init Fail!!!\r\n");
  // SPI.setClockDivider(SPI_CLOCK_DIV2);         // Set SPI to run at 8MHz (16MHz / 2 = 8 MHz)

  for (int i = 0; i < MAX_MGID; i++) {
    driver[i].init(&CAN0);
    driver[i].motor_on();
  }

  for (int i = 0; i < MAX_MGID_1; i++) {
    driver_1[i].init(&CAN1);
    driver_1[i].motor_on();
    driver_1[i].multi_loop_angle_with_speed_control(9.0, 1.0);
  }
  //^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub1, &node);
  rcl_publisher_fini(&pub3, &node);
  rcl_subscription_fini(&sub1, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {
  // Micro_ROS
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  delay(2000);

  state = WAITING_AGENT;

  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}

float mymap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float rpmtoradps(float rpm) {
  return ((rpm * 2 * PI) / 60) * 6;
}

float radpstorpm(float radps) {
  return (radps / 6) * (30 / PI);
}

// static unsigned long previousMillis[2] = { 0, 0 };
// static unsigned long currentMillis;
// MG_Motor
void drivemotor_steering(float angular_mps, unsigned long interval, float wheel_speed_mps) {
  // currentMillis = millis();
  // exposit
  // data = my_flash_store.read();
  static float increment = 2.0;
  static float angle_angular_z[4] = { 0.0, 0.0, 0.0, 0.0 };
  static float angle_angular_z_pre[4] = { 9.0, 9.0, 9.0, 9.0 };
  // static float angle_angular_z_pre[4] = { data.position[0], data.position[1], data.position[2], data.position[3] };

  for (int i = 0; i < 2; i++) {
    angle_angular_z[i] = round(mymap(angular_mps, -1.0, 1.0, 0.0, 18.0));
    // driver_1[i].multi_loop_angle_with_speed_control(angle_angular_z[i], 1.0);
    // previousMillis[i] = currentMillis;
    if (angle_angular_z[i] > angle_angular_z_pre[i]) {
      angle_angular_z_pre[i] += increment;
    } else if (angle_angular_z[i] < angle_angular_z_pre[i]) {
      angle_angular_z_pre[i] -= increment;
    }
    driver_1[i].multi_loop_angle_with_speed_control(angle_angular_z_pre[i], 1.0);

    // if ((angle_angular_z[i] - angle_angular_z_pre[i]) < 0.0) {
    //   driver_1[i].single_loop_angle_with_speed_control(0x01, angle_angular_z[i], rpmtoradps(angular_speeds));
    //   angle_angular_z_pre[i] = angle_angular_z[i];
    // } else if ((angle_angular_z[i] - angle_angular_z_pre[i]) > 0.0) {
    //   driver_1[i].single_loop_angle_with_speed_control(0x00, angle_angular_z[i], rpmtoradps(angular_speeds));
    //   angle_angular_z_pre[i] = angle_angular_z[i];
    // }
    // data.position[i] = angle_angular_z[i];
  }
  // my_flash_store.write(data);

  static float speed_goal;
  speed_goal = rpmtoradps(mymap(wheel_speed_mps, -1.0, 1.0, -300, 300));
  if (abs(speed_goal) > 0.0) {
    driver[0].speed_closed_loop_control(speed_goal);
    driver[1].speed_closed_loop_control(speed_goal);
    driver[2].speed_closed_loop_control(-speed_goal);
    driver[3].speed_closed_loop_control(-speed_goal);
  } else {
    driver[0].stop_motor();
    driver[1].stop_motor();
    driver[2].stop_motor();
    driver[3].stop_motor();
  }
  //^^^^^^^^^^ exposit ^^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void drivemotor_steering_arkerman(float angular_mps, float angular_speeds, float wheel_speed_mps) {
  //arkerman
  // data = my_flash_store.read();
  static float angle_angular_z[4] = { 0.0, 0.0, 0.0, 0.0 };
  // static float angle_angular_zx[] = { 0.0, 0.0, 0.0, 0.0 };
  // static float angle_angular_z_pre[4] = { data.position[0], data.position[1], data.position[2], data.position[3] };
  static float angle_angular_z_pre[4] = { 0.0, 0.0, 0.0, 0.0 };
  static float side_b1 = 0.0;
  static float side_b2 = 0.0;
  static float insideAngle = 0.0;
  static float outsideAngle = 0.0;
  static float insideAngleOutput = 0.0;
  static float outsideAngleOutput = 0.0;
  static float insideVel = 0.0;
  static float outsideVel = 0.0;

  // side_b1 = mymap(abs(angle_angular_zx[i]), (9.0 - START_THREDTHOLD), 0.00, MIN_ICR, MAX_ICR);
  // side_b2 = mymap(abs(angle_angular_zx[i]), (9.0 - START_THREDTHOLD), 0.00, MIN_ICR + WHEEL_WIDESIDE, MAX_ICR + WHEEL_WIDESIDE);
  side_b1 = mymap(abs(angular_mps), (1.0 - START_THREDTHOLD), 0.00, MIN_ICR, MAX_ICR);
  side_b2 = mymap(abs(angular_mps), (1.0 - START_THREDTHOLD), 0.00, MIN_ICR + WHEEL_WIDESIDE, MAX_ICR + WHEEL_WIDESIDE);
  insideAngle = atan((WHEEL_LONGSIDE / 2.0) / (side_b1));
  insideAngle = insideAngle * (180.0 / PI);
  outsideAngle = atan((WHEEL_LONGSIDE / 2.0) / (side_b2));
  outsideAngle = outsideAngle * (180.0 / PI);

  insideAngle = insideAngle / 10.0;
  outsideAngle = outsideAngle / 10.0;

  if (abs(angular_mps) <= START_THREDTHOLD) {
    insideAngleOutput = 0.0;
    outsideAngleOutput = 0.0;
  } else if (angular_mps < -START_THREDTHOLD) {
    insideAngleOutput = outsideAngle * -1.0;
    outsideAngleOutput = insideAngle * -1.0;
  } else if (angular_mps > START_THREDTHOLD) {
    insideAngleOutput = insideAngle;
    outsideAngleOutput = outsideAngle;
  }

  if (insideAngleOutput > 9.0) {
    insideAngleOutput = 9.0;
  } else if (insideAngleOutput < -9.0) {
    insideAngleOutput = -9.0;
  }

  if (outsideAngleOutput > 9.0) {
    outsideAngleOutput = 9.0;
  } else if (outsideAngleOutput < -9.0) {
    outsideAngleOutput = -9.0;
  }

  angle_angular_z[0] = constrain(SERVO_FL_CENTER - insideAngleOutput, 0.0, 18.0);
  angle_angular_z[1] = constrain(SERVO_FR_CENTER - outsideAngleOutput, 0.0, 18.0);

  insideVel = float(rpmtoradps(mymap(wheel_speed_mps, -1.0, 1.0, -300, 300)) * outsideAngle);
  outsideVel = float(rpmtoradps(mymap(wheel_speed_mps, -1.0, 1.0, -300, 300)) * insideAngle);
  insideVel = constrain(insideVel, -rpmtoradps(300), rpmtoradps(300));
  outsideVel = constrain(outsideVel, -rpmtoradps(300), rpmtoradps(300));
  // angle_angular_z[0] = mymap(cmd_msg[1], -0.4, 0.4, 0.0, 18.0);
  // angle_angular_z[1] = mymap(cmd_msg[1], -0.4, 0.4, 0.0, 18.0);
  for (int i = 0; i < 2; i++) {
    driver_1[i].multi_loop_angle_with_speed_control(angle_angular_z[i], 1.0);

    // if ((angle_angular_z[i] - angle_angular_z_pre[i]) < 0.0) {
    //   driver_1[i].single_loop_angle_with_speed_control(0x01, angle_angular_z[i], rpmtoradps(angular_speeds));
    //   angle_angular_z_pre[i] = angle_angular_z[i];
    // } else if ((angle_angular_z[i] - angle_angular_z_pre[i]) > 0.0) {
    //   driver_1[i].single_loop_angle_with_speed_control(0x00, angle_angular_z[i], rpmtoradps(angular_speeds));
    //   angle_angular_z_pre[i] = angle_angular_z[i];
    // }
    // data.position[i] = angle_angular_z[i];
  }
  // my_flash_store.write(data);

  if (abs(insideVel) > 0.0) {
    driver[0].speed_closed_loop_control(insideVel);
    driver[2].speed_closed_loop_control(-insideVel);
  } else {
    driver[0].stop_motor();
    driver[2].stop_motor();
  }
  if (abs(outsideVel) > 0.0) {
    driver[1].speed_closed_loop_control(outsideVel);
    driver[3].speed_closed_loop_control(-outsideVel);
  } else {
    driver[1].stop_motor();
    driver[3].stop_motor();
  }
  //^^^^^^^^^^ arkerman ^^^^^^^^^^^^^^^^^^^^^^^^^^^
}
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// AK_Motor

//^^^^^^^^^^^^^^^ AK_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^