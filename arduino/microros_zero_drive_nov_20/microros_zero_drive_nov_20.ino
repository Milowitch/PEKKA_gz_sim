// MG_Motor
#include <Arduino.h>
#include <math.h>
#include <ArduinoJson.h>
// #include <FlashStorage.h>
#include <vector>
#include "MGDrive.h"
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// SM_Motor
#include <SCServo.h>
//^^^^^^^^^^^^^^^ SM_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

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
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

float calculateRPM(float speed_mps, float diameter);
float roundToIncrement(float value, float increment);
float mymap(float x, float in_min, float in_max, float out_min, float out_max);
float rpmtoradps(float rpm);
float radpstorpm(float radps);
void drivemotor_steering(float angular_radps, float maxSpeedRPM, float wheel_speed_mps);
void drivemotor_steering_arkerman(float angular_radps, float maxSpeedRPM, float wheel_speed_mps);
void drivemotor_steering_donut(float angular_radps, float maxSpeedRPM, float wheel_speed_mps);

// MG_Motor
// init MCP_CAN object
MCP_CAN CAN0(9);
// MCP_CAN CAN1(10);

#define CAN0_INT 2
#define CAN1_INT 3

MGDrive mgDrive_wheel(&CAN0, CAN0_INT);
// MGDrive mgDrive_str(&CAN1, CAN1_INT);

// #define MAX_MGID_WHEEL 4
#define MOT_CAN_ID_FR 2
#define MOT_CAN_ID_FL 1
#define MOT_CAN_ID_RR 4
#define MOT_CAN_ID_RL 3

// #define MAX_ID_STR 4
#define MOT_ID_STRFR 7
#define MOT_ID_STRFL 8
#define MOT_ID_STRRR 5
#define MOT_ID_STRRL 6

#define MIN_ICR 0.43f         // m
#define MAX_ICR 2.58f         // m WHEEL_WIDESIDE * 6
#define WHEEL_LONGSIDE 0.61f  // m
#define WHEEL_WIDESIDE 0.43f  // m
#define WHEEL_DIMITER 0.254f  // m เส้นผ่านศูนย์กลาง
#define SERVO_STRFL_MIN_ANGLE 0.0f
#define SERVO_STRFL_MAX_ANGLE 180.0f
#define SERVO_STRFL_CENTER 90.0f
#define SERVO_STRFR_MIN_ANGLE 0.0f
#define SERVO_STRFR_MAX_ANGLE 180.0f
#define SERVO_STRFR_CENTER 90.0f
#define SERVO_STRRL_MIN_ANGLE 0.0f
#define SERVO_STRRL_MAX_ANGLE 180.0f
#define SERVO_STRRL_CENTER 90.0f
#define SERVO_STRRR_MIN_ANGLE 0.0f
#define SERVO_STRRR_MAX_ANGLE 180.0f
#define SERVO_STRRR_CENTER 90.0f
#define START_THREDTHOLD 0.01f

// left_front, right_front, left_rear, right_rear
const float increment = 0.1f;
const uint8_t MG_WHEEL_ID[] = { 
  MOT_CAN_ID_FL,
  MOT_CAN_ID_FR,
  MOT_CAN_ID_RL,
  MOT_CAN_ID_RR
};
// const uint8_t MG_WHEEL_ID[] = { MOT_CAN_ID_FL };
const int MG_WHEEL_ID_SIZE = sizeof(MG_WHEEL_ID) / sizeof(MG_WHEEL_ID[0]);
uint8_t MG_STR_ID[] = { 
  MOT_ID_STRFL,
  MOT_ID_STRFR,
  MOT_ID_STRRL,
  MOT_ID_STRRR
};
const int MG_STR_ID_SIZE = sizeof(MG_STR_ID) / sizeof(MG_STR_ID[0]);
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// SM_Motor
#define SM_STRFR_POINT_MIN 0
#define SM_STRFR_POINT_MAX 2048

#define SM_STRFL_POINT_MIN 2048
#define SM_STRFL_POINT_MAX 4095

#define SM_STRRR_POINT_MIN 2049
#define SM_STRRR_POINT_MAX 4095

#define SM_STRRL_POINT_MIN 0
#define SM_STRRL_POINT_MAX 2048

SMSCL sm;

// byte ID[] = { MOT_ID_STRFR, MOT_ID_STRFL, MOT_ID_STRRR, MOT_ID_STRRL };
// const int ID_SIZE = sizeof(ID) / sizeof(ID[0]);
u16 Speed[] = { 2250, 2250, 2250, 2250 };
byte ACC[] = { 50, 50, 50, 50 };
//^^^^^^^^^^^^^^^ SM_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

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

std_msgs__msg__Float64MultiArray cmd_msg;
// std_msgs__msg__Int8MultiArray msg;
std_msgs__msg__String motor_msg;
std_msgs__msg__String fb_msg;

#define LED_PIN A0
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

const int capacity = JSON_ARRAY_SIZE(8) + 8 * JSON_OBJECT_SIZE(8);
StaticJsonDocument<capacity> doc;
char char_motor_msg[capacity];

const int fb_capacity = JSON_ARRAY_SIZE(8);
StaticJsonDocument<fb_capacity> fd_doc;
char char_fb_msg[fb_capacity];

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    // pub_motor_msg
    serializeJson(doc, char_motor_msg, sizeof(char_motor_msg));
    rosidl_runtime_c__String__assign(&motor_msg.data, char_motor_msg);
    rcl_publish(&pub1, &motor_msg, NULL);

    serializeJson(fd_doc, char_fb_msg, sizeof(char_fb_msg));
    rosidl_runtime_c__String__assign(&fb_msg.data, char_fb_msg);
    rcl_publish(&pub3, &fb_msg, NULL);
    //^^^^^^^^^^ pub_motor_msg ^^^^^^^^^^^^^^^^^^^^^^
  }
}

unsigned long currentmilli = millis();
unsigned long premilli = currentmilli;
bool LEDstatus = false;
unsigned long ros_cmd_callback_last = millis();

const char *stateStrings[] = {
  "WAITING_AGENT",
  "AGENT_AVAILABLE",
  "AGENT_CONNECTED",
  "AGENT_DISCONNECTED"
};

//message cb
void ros_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  ros_cmd_callback_last = millis();
  // static float cmd_msg[8];
  // for (int i = 0; i < msg->data.capacity; i++) {
  //   cmd_msg[i] = msg->data.data[i];
  // }

  const float angle_speed = 60.0;
  std::vector<float> vel_goal = {
    constrain(msg->data.data[0], -300.0f, 300.0f),
    constrain(msg->data.data[1], -300.0f, 300.0f),
    constrain(msg->data.data[2], -300.0f, 300.0f),
    constrain(msg->data.data[3], -300.0f, 300.0f)
  };
  float myspeed = 0.0;
  s16 Position[4];
  const float my_range = PI / 2.0f;
  // const float my_range = 1.571;
  Position[0] = constrain(mymap(constrain(msg->data.data[4], -my_range, my_range), -my_range, my_range, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX), SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
  Position[1] = constrain(mymap(constrain(msg->data.data[5], -my_range, my_range), -my_range, my_range, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX), SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
  Position[2] = constrain(mymap(constrain(msg->data.data[6], -my_range, my_range), -my_range, my_range, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX), SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);
  Position[3] = constrain(mymap(constrain(msg->data.data[7], -my_range, my_range), -my_range, my_range, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX), SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);

  if (abs(vel_goal[0]) > 0.0) {
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FL, -vel_goal[0]);
  } else {
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FL);
  }
  if (abs(vel_goal[1]) > 0.0) {
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FR, vel_goal[1]);
  } else {
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FR);
  }
  if (abs(vel_goal[2]) > 0.0) {
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RL, -vel_goal[2]);
  } else {
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RL);
  }
  if (abs(vel_goal[3]) > 0.0) {
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RR, vel_goal[3]);
  } else {
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RR);
  }
  sm.SyncWritePosEx(MG_STR_ID, MG_STR_ID_SIZE, Position, Speed, ACC);

  fd_doc[0] = -vel_goal[0];
  fd_doc[1] = vel_goal[1];
  fd_doc[2] = -vel_goal[2];
  fd_doc[3] = vel_goal[3];
  fd_doc[4] = Position[0];
  fd_doc[5] = Position[1];
  fd_doc[6] = Position[2];
  fd_doc[7] = Position[3];

  for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
    doc[i]["name"] = "mg_" + String(MG_WHEEL_ID[i]);
    doc[i]["effort"] = mgDrive_wheel.getTorqueCurrentValue(MG_WHEEL_ID[i]);
    doc[i]["position"] = mgDrive_wheel.getEncoderPositionValue(MG_WHEEL_ID[i]);
    doc[i]["tempareture"] = mgDrive_wheel.getTemperatureValue(MG_WHEEL_ID[i]);
    doc[i]["velocity"] = mgDrive_wheel.getSpeedValue(MG_WHEEL_ID[i]);
  }

  for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
    int Pos[MG_STR_ID_SIZE];
    int Speed[MG_STR_ID_SIZE];
    int Load[MG_STR_ID_SIZE];
    int Voltage[MG_STR_ID_SIZE];
    int Temper[MG_STR_ID_SIZE];
    int Move[MG_STR_ID_SIZE];
    int Current[MG_STR_ID_SIZE];

    doc[i + (MG_STR_ID_SIZE)]["name"] = "sm_" + String(MG_STR_ID[i]);
    if (sm.FeedBack(MG_STR_ID[i]) != -1) {
      Pos[i] = sm.ReadPos(-1);
      Speed[i] = sm.ReadSpeed(-1);
      Load[i] = sm.ReadLoad(-1);
      Voltage[i] = sm.ReadVoltage(-1);
      Temper[i] = sm.ReadTemper(-1);
      Move[i] = sm.ReadMove(-1);
      Current[i] = sm.ReadCurrent(-1);

      doc[i + (MG_STR_ID_SIZE)]["effort"] = Load[i];
      doc[i + (MG_STR_ID_SIZE)]["position"] = Pos[i];
      doc[i + (MG_STR_ID_SIZE)]["tempareture"] = Temper[i];
      doc[i + (MG_STR_ID_SIZE)]["velocity"] = Speed[i];
      doc[i + (MG_STR_ID_SIZE)]["voltage"] = Voltage[i];
      doc[i + (MG_STR_ID_SIZE)]["current"] = Current[i];
      doc[i + (MG_STR_ID_SIZE)]["move"] = Move[i];
    }
  }
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

  // static float memory[8];
  cmd_msg.data.capacity = 8;
  cmd_msg.data.size = 0;
  cmd_msg.data.data = (double *)malloc(cmd_msg.data.capacity * sizeof(double));

  cmd_msg.layout.dim.capacity = 8;
  cmd_msg.layout.dim.size = 0;
  cmd_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(cmd_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < cmd_msg.layout.dim.capacity; i++) {
    cmd_msg.layout.dim.data[i].label.capacity = 5;
    cmd_msg.layout.dim.data[i].label.size = 0;
    cmd_msg.layout.dim.data[i].label.data = (char *)malloc(cmd_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // ENSURE HERE THAT ALL THE MEMBERS ARE INIT RECURSIVELY, I HAVE JUST COPIED YOUR CODE

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // unsigned int rcl_wait_timeout = 1000;  // in ms
  // RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub1, &cmd_msg, &ros_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // rosidl_runtime_c__String__init(&wh_msg.data);

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub1, &node);
  // rcl_publisher_fini(&pub3, &node);
  rcl_subscription_fini(&sub1, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {
  // SM_Motor
  Serial1.begin(115200);
  sm.pSerial = &Serial1;
  SerialUSB.begin(115200);
  //^^^^^^^^^^^^^^^ SM_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

  // Micro_ROS
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  // MG_Moto
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    // Serial.print("CAN0: Init OK!\r\n");
    CAN0.setMode(MCP_NORMAL);

    for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
      mgDrive_wheel.Motor_stop(MG_STR_ID[i]);
      mgDrive_wheel.Clearmotorerrorstate(MG_WHEEL_ID[i]);
      mgDrive_wheel.Motor_on(MG_WHEEL_ID[i]);
    }
  }
  //^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

  delay(2000);

  state = WAITING_AGENT;

  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void loop() {
  currentmilli = millis();
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

    // if (currentmilli - premilli >= 500) {
    //   premilli = currentmilli;

    //   SerialUSB.println(stateStrings[state]);
    // }
  } else {
    // if (currentmilli - premilli >= 500) {
    //   premilli = currentmilli;

    //   SerialUSB.println(stateStrings[state]);

    //   LEDstatus = !LEDstatus;
    //   digitalWrite(LED_PIN, LEDstatus);
    // }
  }

  // if ((currentmilli - ros_cmd_callback_last) >= 1000) {
  //   for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
  //     mgDrive_wheel.Motor_stop(MG_STR_ID[i]);
  //     mgDrive_wheel.Clearmotorerrorstate(MG_WHEEL_ID[i]);
  //     mgDrive_wheel.Motor_off(MG_WHEEL_ID[i]);
  //   }
  // }
}

float roundToIncrement(float value, float increment) {
  return round(value / increment) * increment;
}

float mymap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float rpmtoradps(float rpm) {
  return ((rpm * 2.0 * PI) / 60.0) * 6.0;
}

float radpstorpm(float radps) {
  return (radps / 6.0) * (30.0 / PI);
}

// MG_Motor
float calculateRPM(float speed_mps, float diameter) {
  return (speed_mps * 60.0) / (2.0 * PI * (diameter / 2.0));
}

//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// AK_Motor

//^^^^^^^^^^^^^^^ AK_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^
