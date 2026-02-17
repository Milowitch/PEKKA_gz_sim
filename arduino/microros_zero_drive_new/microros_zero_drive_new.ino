// MG_Motor
#include <Arduino.h>
#include <math.h>
#include <ArduinoJson.h>
// #include <FlashStorage.h>
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
#include <std_msgs/msg/float64_multi_array.h>
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
#define MOT_ID_STRFR 6
#define MOT_ID_STRFL 8
#define MOT_ID_STRRR 5
#define MOT_ID_STRRL 7

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

const float increment = 0.1f;
const uint8_t MG_WHEEL_ID[] = { MOT_CAN_ID_FR, MOT_CAN_ID_FL, MOT_CAN_ID_RR, MOT_CAN_ID_RL };
// const uint8_t MG_WHEEL_ID[] = { MOT_CAN_ID_FL };
const int MG_WHEEL_ID_SIZE = sizeof(MG_WHEEL_ID) / sizeof(MG_WHEEL_ID[0]);
uint8_t MG_STR_ID[] = { MOT_ID_STRFR, MOT_ID_STRFL, MOT_ID_STRRR, MOT_ID_STRRL };
const int MG_STR_ID_SIZE = sizeof(MG_STR_ID) / sizeof(MG_STR_ID[0]);
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// SM_Motor
#define SM_STRFR_POINT_MIN 0
#define SM_STRFR_POINT_MAX 2048

#define SM_STRFL_POINT_MIN 2049
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

const int capacity = JSON_ARRAY_SIZE(8) + 8 * JSON_OBJECT_SIZE(8);
StaticJsonDocument<capacity> doc;
char char_motor_msg[capacity];

const int fb_capacity = JSON_ARRAY_SIZE(6);
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

const char* stateStrings[] = {
  "WAITING_AGENT",
  "AGENT_AVAILABLE",
  "AGENT_CONNECTED",
  "AGENT_DISCONNECTED"
};

//message cb
void ros_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  ros_cmd_callback_last = millis();
  static float cmd_msg[6];
  for (int i = 0; i < msg->data.capacity; i++) {
    cmd_msg[i] = msg->data.data[i];
  }

  if (cmd_msg[3] == 1.0) {
    // for (int i = 1; i <= MAX_MGID_WHEEL; i++) {
    //   mgDrive_wheel.Clearmotorerrorstate(i);
    //   mgDrive_wheel.Motor_on(i);
    // }
    for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
      mgDrive_wheel.Clearmotorerrorstate(MG_WHEEL_ID[i]);
      mgDrive_wheel.Motor_on(MG_WHEEL_ID[i]);
    }
    // for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
    //   mgDrive_str.Clearmotorerrorstate(MG_STR_ID[i]);
    //   mgDrive_str.Motor_on(MG_STR_ID[i]);
    // }
  }

  if (cmd_msg[4] == 1.0) {
    // for (int i = 1; i <= MAX_MGID_WHEEL; i++) {
    //   mgDrive_wheel.Motor_stop(i);
    //   mgDrive_wheel.Motor_off(i);
    // }
    for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
      mgDrive_wheel.Motor_stop(MG_WHEEL_ID[i]);
      mgDrive_wheel.Motor_off(MG_WHEEL_ID[i]);
    }
    // for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
    //   mgDrive_str.Motor_stop(MG_STR_ID[i]);
    //   mgDrive_str.Motor_off(MG_STR_ID[i]);
    // }
  }

  const float angle_speed = 60.0;

  if (cmd_msg[2] == 0.0) {
    drivemotor_steering(cmd_msg[1], angle_speed, cmd_msg[0]);
  } else if (cmd_msg[2] == 1.0) {
    drivemotor_steering_arkerman(cmd_msg[1], angle_speed, cmd_msg[0]);
  } else if (cmd_msg[2] == 2.0) {
    drivemotor_steering_donut(cmd_msg[1], angle_speed, cmd_msg[0]);
  } else if (cmd_msg[2] == 3.0) {
    float angle_angular_z = 0.0;
    angle_angular_z = mymap(cmd_msg[1], -1.0, 1.0, 0.0, 180.0);
    s16 Position[4];
    if (cmd_msg[5] == MOT_ID_STRFR) {
      // mgDrive_str.Multiloopanglecontrol2(MOT_ID_STRFR, angle_speed, angle_angular_z);
      Position[0] = mymap(angle_angular_z, 0, 180, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
      Position[1] = mymap(90, 0, 180, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
      Position[2] = mymap(90, 0, 180, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
      Position[3] = mymap(90, 0, 180, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);
    } else if (cmd_msg[5] == MOT_ID_STRFL) {
      // mgDrive_str.Multiloopanglecontrol2(MOT_ID_STRFL, angle_speed, angle_angular_z);
      Position[1] = mymap(angle_angular_z, 0.0, 180.0, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
      Position[0] = mymap(90, 0, 180, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
      Position[2] = mymap(90, 0, 180, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
      Position[3] = mymap(90, 0, 180, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);
    } else if (cmd_msg[5] == MOT_ID_STRRR) {
      // mgDrive_str.Multiloopanglecontrol2(MOT_ID_STRRR, angle_speed, angle_angular_z);
      Position[2] = mymap(angle_angular_z, 0.0, 180.0, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
      Position[0] = mymap(90, 0, 180, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
      Position[1] = mymap(90, 0, 180, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
      Position[3] = mymap(90, 0, 180, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);
    } else if (cmd_msg[5] == MOT_ID_STRRL) {
      // mgDrive_str.Multiloopanglecontrol2(MOT_ID_STRRL, angle_speed, angle_angular_z);
      Position[3] = mymap(angle_angular_z, 0.0, 180.0, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);
      Position[0] = mymap(90, 0, 180, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
      Position[1] = mymap(90, 0, 180, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
      Position[2] = mymap(90, 0, 180, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
    } else {
      float speed_goal;
      float myspeed = 0.0;

      Position[0] = mymap(90, 0, 180, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
      Position[1] = mymap(90, 0, 180, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
      Position[2] = mymap(90, 0, 180, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
      Position[3] = mymap(90, 0, 180, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);

      speed_goal = calculateRPM(cmd_msg[0], WHEEL_DIMITER);
      // speed_goal = roundToIncrement(calculateRPM(cmd_msg[0], WHEEL_DIMITER), increment);
      if (abs(speed_goal) > 0.0) {
        // if (speed_goal > myspeed) {
        //   myspeed += increment;
        // } else if (speed_goal < myspeed) {
        //   myspeed -= increment;
        // }
        mgDrive_wheel.Speedclosedloopcontrol(cmd_msg[5], speed_goal);
      } else {
        myspeed = 0.0;
        for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
          mgDrive_wheel.Motor_stop(MG_WHEEL_ID[i]);
          mgDrive_wheel.ReadMotorState2(MG_WHEEL_ID[i]);
        }
      }
    }

    // sm.SyncWritePosEx(ID, MG_STR_ID_SIZE, Position, Speed, ACC);
    sm.SyncWritePosEx(MG_STR_ID, MG_STR_ID_SIZE, Position, Speed, ACC);
  }

  // for (int i = 1; i <= MAX_MGID_WHEEL; i++) {
  //   doc[i]["name"] = "mg_" + String(i);
  //   doc[i]["effort"] = mgDrive_wheel.getTorqueCurrentValue(i);
  //   doc[i]["position"] = mgDrive_wheel.getEncoderPositionValue(i);
  //   doc[i]["tempareture"] = mgDrive_wheel.getTemperatureValue(i);
  //   doc[i]["velocity"] = mgDrive_wheel.getSpeedValue(i);
  // }
  for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
    doc[i]["name"] = "mg_" + String(MG_WHEEL_ID[i]);
    doc[i]["effort"] = mgDrive_wheel.getTorqueCurrentValue(MG_WHEEL_ID[i]);
    doc[i]["position"] = mgDrive_wheel.getEncoderPositionValue(MG_WHEEL_ID[i]);
    doc[i]["tempareture"] = mgDrive_wheel.getTemperatureValue(MG_WHEEL_ID[i]);
    doc[i]["velocity"] = mgDrive_wheel.getSpeedValue(MG_WHEEL_ID[i]);
  }
  // for (int i = 5; i <= (MAX_ID_STR + 4); i++) {
  //   doc[i]["name"] = "mg_" + String(i);
  //   doc[i]["effort"] = mgDrive_str.getTorqueCurrentValue(i);
  //   doc[i]["position"] = mgDrive_str.getEncoderPositionValue(i);
  //   doc[i]["tempareture"] = mgDrive_str.getTemperatureValue(i);
  //   doc[i]["velocity"] = mgDrive_str.getSpeedValue(i);
  // }
  // for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
  //   doc[i]["name"] = "mg_" + String(MG_STR_ID[i]);
  //   doc[i]["effort"] = mgDrive_str.getTorqueCurrentValue(MG_STR_ID[i]);
  //   doc[i]["position"] = mgDrive_str.getEncoderPositionValue(MG_STR_ID[i]);
  //   doc[i]["tempareture"] = mgDrive_str.getTemperatureValue(MG_STR_ID[i]);
  //   doc[i]["velocity"] = mgDrive_str.getSpeedValue(MG_STR_ID[i]);
  // }

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
  // SM_Motor
  Serial1.begin(115200);
  sm.pSerial = &Serial1;
  // SerialUSB.begin(115200);
  // sm.pSerial = &SerialUSB;
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
  } else {
  }
  // else Serial.print("CAN0: Init Fail!!!\r\n");

  // if (CAN1.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
  // Serial.print("CAN1: Init OK!\r\n");
  // CAN1.setMode(MCP_NORMAL);
  // }
  // else Serial.print("CAN1: Init Fail!!!\r\n");
  // SPI.setClockDivider(SPI_CLOCK_DIV2);         // Set SPI to run at 8MHz (16MHz / 2 = 8 MHz)

  // for (int i = 1; i <= MAX_MGID_WHEEL; i++) {
  //   mgDrive_wheel.Motor_stop(i);
  //   mgDrive_wheel.Clearmotorerrorstate(i);
  //   mgDrive_wheel.Motor_on(i);
  // }

  // for (int i = 5; i <= (MAX_ID_STR + 4); i++) {
  //   mgDrive_str.Motor_stop(i);
  //   mgDrive_str.Clearmotorerrorstate(i);
  //   mgDrive_str.Motor_on(i);
  // }

  // for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
  //   mgDrive_str.Motor_stop(MG_STR_ID[i]);
  //   mgDrive_str.Clearmotorerrorstate(MG_STR_ID[i]);
  //   mgDrive_str.Motor_on(MG_STR_ID[i]);
  // }
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

    if (currentmilli - premilli >= 500) {
      premilli = currentmilli;

      // SerialUSB.println(stateStrings[state]);
    }
  } else {
    if (currentmilli - premilli >= 500) {
      premilli = currentmilli;

      // SerialUSB.println(stateStrings[state]);

      LEDstatus = !LEDstatus;
      digitalWrite(LED_PIN, LEDstatus);
    }
  }

  if((currentmilli - ros_cmd_callback_last) >= 1000){
    for (uint8_t i = 0; i < MG_WHEEL_ID_SIZE; i++) {
      mgDrive_wheel.Motor_stop(MG_STR_ID[i]);
      mgDrive_wheel.Clearmotorerrorstate(MG_WHEEL_ID[i]);
      mgDrive_wheel.Motor_off(MG_WHEEL_ID[i]);
    }
  }
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
void drivemotor_steering(float angular_radps, float maxSpeedRPM, float wheel_speed_mps) {
  // exposit
  // float angle_angular_z[4] = { 0.0, 0.0, 0.0, 0.0 };
  s16 Position[4];

  // for (int i = 5; i <= (MAX_ID_STR + 4); i++) {
  //   angle_angular_z[i - 5] = mymap(angular_radps, -1.0, 1.0, 0.0, 180.0);
  //   mgDrive_str.Multiloopanglecontrol2(i, maxSpeedRPM, angle_angular_z[i - 5]);

  //   Position[i - 5] = mymap(angle_angular_z[i - 5], 0, 180, SM_PWM_MIN, SM_PWM_MAX);
  // }

  // for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
  // angle_angular_z[i] = mymap(angular_radps, -1.0, 1.0, 0.0, 180.0);

  // mgDrive_str.Multiloopanglecontrol2(MG_STR_ID[i], maxSpeedRPM, angle_angular_z[i]);

  // Position[i] = mymap(angle_angular_z[i], 0, 180, SM_PWM_MIN, SM_PWM_MAX);
  // }

  Position[0] = mymap(angular_radps, -1.0, 1.0, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
  Position[1] = mymap(angular_radps, -1.0, 1.0, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
  Position[2] = mymap(angular_radps, -1.0, 1.0, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
  Position[3] = mymap(angular_radps, -1.0, 1.0, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);

  // sm.SyncWritePosEx(ID, MG_STR_ID_SIZE, Position, Speed, ACC);
  sm.SyncWritePosEx(MG_STR_ID, MG_STR_ID_SIZE, Position, Speed, ACC);

  // float myspeed = 0.0;
  float speed_goal = calculateRPM(wheel_speed_mps, WHEEL_DIMITER);
  // speed_goal = roundToIncrement(calculateRPM(wheel_speed_mps, WHEEL_DIMITER), increment);
  if (abs(speed_goal) > 0.0) {
    // if (speed_goal > myspeed) {
    //   myspeed += increment;
    // } else if (speed_goal < myspeed) {
    //   myspeed -= increment;
    // }
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FL, -speed_goal);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FR, speed_goal);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RL, -speed_goal);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RR, speed_goal);

  } else {
    // myspeed = 0.0;
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FL);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FR);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RL);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RR);
  }

  fd_doc[0] = speed_goal;
  fd_doc[1] = -1;
  fd_doc[2] = -1;
  // fd_doc[0] = cmd_msg[0];
  // fd_doc[1] = cmd_msg[1];
  // fd_doc[2] = cmd_msg[2];
  // fd_doc[3] = cmd_msg[3];
  // fd_doc[4] = cmd_msg[4];
  // fd_doc[5] = cmd_msg[5];

  // fd_doc[6] = angle_angular_z_pre[0];
  // fd_doc[7] = angle_angular_z_pre[1];
  // for (int i = 0; i < 2; i++) {
  // fd_doc[i] = motor_status_1[i].position;
  // }

  //^^^^^^^^^^ exposit ^^^^^^^^^^^^^^^^^^^^^^^^^^^
}

float calculateRPM(float speed_mps, float diameter) {
  return (speed_mps * 60.0) / (2.0 * PI * (diameter / 2.0));
}

void drivemotor_steering_arkerman(float angular_radps, float maxSpeedRPM, float wheel_speed_mps) {
  //arkerman
  float angle_angular_z[4] = { 0.0, 0.0, 0.0, 0.0 };
  float insideAngleOutput = 0.0;
  float outsideAngleOutput = 0.0;
  float insideVelOutput = 0.0;
  float outsideVelOutput = 0.0;

  float side_b1 = mymap(abs(angular_radps), (1.0 - START_THREDTHOLD), 0.00, MIN_ICR, MAX_ICR);
  float side_b2 = mymap(abs(angular_radps), (1.0 - START_THREDTHOLD), 0.00, MIN_ICR + WHEEL_WIDESIDE, MAX_ICR + WHEEL_WIDESIDE);
  float insideAngle = (atan((WHEEL_LONGSIDE / 2.0) / (side_b1))) * (180.0 / PI);
  float outsideAngle = (atan((WHEEL_LONGSIDE / 2.0) / (side_b2))) * (180.0 / PI);

  float Rwheelinside = sqrt((side_b1 * side_b1) + (WHEEL_LONGSIDE * WHEEL_LONGSIDE));
  float Rwheeloutside = sqrt((side_b2 * side_b2) + (WHEEL_LONGSIDE * WHEEL_LONGSIDE));

  // insideVel = roundToIncrement(calculateRPM(wheel_speed_mps * (Rwheelinside / (side_b1 + (WHEEL_WIDESIDE / 2))), WHEEL_DIMITER), increment);
  // outsideVel = roundToIncrement(calculateRPM(wheel_speed_mps * (Rwheeloutside / (side_b2 - (WHEEL_WIDESIDE / 2))), WHEEL_DIMITER), increment);
  float insideVel = calculateRPM(wheel_speed_mps * (Rwheelinside / (side_b1 + (WHEEL_WIDESIDE / 2))), WHEEL_DIMITER);
  float outsideVel = calculateRPM(wheel_speed_mps * (Rwheeloutside / (side_b2 - (WHEEL_WIDESIDE / 2))), WHEEL_DIMITER);
  float speed_goal = calculateRPM(wheel_speed_mps, WHEEL_DIMITER);

  if (abs(angular_radps) <= START_THREDTHOLD) {
    insideAngleOutput = 0.0;
    outsideAngleOutput = 0.0;
    insideVelOutput = constrain(speed_goal, -300.0, 300.0);
    outsideVelOutput = constrain(speed_goal, -300.0, 300.0);
  } else if (angular_radps < -START_THREDTHOLD) {
    insideAngleOutput = constrain(outsideAngle * -1.0, -90.0, 90.0);
    outsideAngleOutput = constrain(insideAngle * -1.0, -90.0, 90.0);
    insideVelOutput = constrain(insideVel, -300.0, 300.0);
    outsideVelOutput = constrain(outsideVel, -300.0, 300.0);
  } else if (angular_radps > START_THREDTHOLD) {
    insideAngleOutput = constrain(insideAngle, -90.0, 90.0);
    outsideAngleOutput = constrain(outsideAngle, -90.0, 90.0);
    insideVelOutput = constrain(outsideVel, -300.0, 300.0);
    outsideVelOutput = constrain(insideVel, -300.0, 300.0);
  }

  angle_angular_z[0] = constrain(SERVO_STRFL_CENTER + insideAngleOutput, SERVO_STRFR_MIN_ANGLE, SERVO_STRFR_MAX_ANGLE);
  angle_angular_z[1] = constrain(SERVO_STRFR_CENTER + outsideAngleOutput, SERVO_STRFL_MIN_ANGLE, SERVO_STRFL_MAX_ANGLE);
  angle_angular_z[2] = constrain(SERVO_STRRL_CENTER - insideAngleOutput, SERVO_STRRR_MIN_ANGLE, SERVO_STRRR_MAX_ANGLE);
  angle_angular_z[3] = constrain(SERVO_STRRR_CENTER - outsideAngleOutput, SERVO_STRRL_MIN_ANGLE, SERVO_STRRL_MAX_ANGLE);

  s16 Position[4];
  Position[0] = mymap(angle_angular_z[0], SERVO_STRFR_MIN_ANGLE, SERVO_STRFR_MAX_ANGLE, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
  Position[1] = mymap(angle_angular_z[1], SERVO_STRFL_MIN_ANGLE, SERVO_STRFL_MAX_ANGLE, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
  Position[2] = mymap(angle_angular_z[2], SERVO_STRRR_MIN_ANGLE, SERVO_STRRR_MAX_ANGLE, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
  Position[3] = mymap(angle_angular_z[3], SERVO_STRRL_MIN_ANGLE, SERVO_STRRL_MAX_ANGLE, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);

  // for (uint8_t i = 0; i < MG_STR_ID_SIZE; i++) {
  // Position[i] = mymap(angle_angular_z[i], 0, 180, SM_PWM_MIN, SM_PWM_MAX);
  // mgDrive_str.Multiloopanglecontrol2(MG_STR_ID[i], maxSpeedRPM, angle_angular_z[i]);
  // }

  // sm.SyncWritePosEx(ID, MG_STR_ID_SIZE, Position, Speed, ACC);
  sm.SyncWritePosEx(MG_STR_ID, MG_STR_ID_SIZE, Position, Speed, ACC);

  // float myinsideVelspeed = 0.0;
  // float myoutsideVelspeed = 0.0;

  if (abs(insideVelOutput) > 0.0) {
    // if (insideVelOutput > myinsideVelspeed) {
    //   myinsideVelspeed += increment;
    // } else if (insideVelOutput < myinsideVelspeed) {
    //   myinsideVelspeed -= increment;
    // }
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FR, insideVelOutput);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RR, insideVelOutput);
  } else {
    // myinsideVelspeed = 0.0;
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FR);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RR);
  }
  if (abs(outsideVelOutput) > 0.0) {
    // if (outsideVelOutput > myoutsideVelspeed) {
    //   myoutsideVelspeed += increment;
    // } else if (outsideVelOutput < myoutsideVelspeed) {
    //   myoutsideVelspeed -= increment;
    // }
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FL, -outsideVelOutput);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RL, -outsideVelOutput);
  } else {
    // myoutsideVelspeed = 0.0;
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FL);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RL);
  }

  fd_doc[0] = -1;
  fd_doc[1] = insideVelOutput;
  fd_doc[2] = outsideVelOutput;
  //^^^^^^^^^^ arkerman ^^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void drivemotor_steering_donut(float angular_radps, float maxSpeedRPM, float wheel_speed_mps) {
  float angle_angular_z[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  float Angle = (atan((WHEEL_LONGSIDE / 2.0f) / (WHEEL_WIDESIDE / 2.0f))) * (180.0f / PI);

  angle_angular_z[0] = constrain(SERVO_STRFL_CENTER + Angle, SERVO_STRFR_MIN_ANGLE, SERVO_STRFR_MAX_ANGLE);
  angle_angular_z[1] = constrain(SERVO_STRFR_CENTER - Angle, SERVO_STRFL_MIN_ANGLE, SERVO_STRFL_MAX_ANGLE);
  angle_angular_z[2] = constrain(SERVO_STRRL_CENTER - Angle, SERVO_STRRR_MIN_ANGLE, SERVO_STRRR_MAX_ANGLE);
  angle_angular_z[3] = constrain(SERVO_STRRR_CENTER + Angle, SERVO_STRRL_MIN_ANGLE, SERVO_STRRL_MAX_ANGLE);

  s16 Position[4];
  Position[0] = mymap(angle_angular_z[0], SERVO_STRFR_MIN_ANGLE, SERVO_STRFR_MAX_ANGLE, SM_STRFR_POINT_MIN, SM_STRFR_POINT_MAX);
  Position[1] = mymap(angle_angular_z[1], SERVO_STRFL_MIN_ANGLE, SERVO_STRFL_MAX_ANGLE, SM_STRFL_POINT_MIN, SM_STRFL_POINT_MAX);
  Position[2] = mymap(angle_angular_z[2], SERVO_STRRR_MIN_ANGLE, SERVO_STRRR_MAX_ANGLE, SM_STRRR_POINT_MIN, SM_STRRR_POINT_MAX);
  Position[3] = mymap(angle_angular_z[3], SERVO_STRRL_MIN_ANGLE, SERVO_STRRL_MAX_ANGLE, SM_STRRL_POINT_MIN, SM_STRRL_POINT_MAX);

  sm.SyncWritePosEx(MG_STR_ID, MG_STR_ID_SIZE, Position, Speed, ACC);

  // float myspeed = 0.0;
  float speed_goal = calculateRPM(angular_radps, WHEEL_DIMITER);
  // speed_goal = roundToIncrement(calculateRPM(angular_radps, WHEEL_DIMITER), increment);
  if (abs(speed_goal) > 0.0) {
    // if (speed_goal > myspeed) {
    //   myspeed += increment;
    // } else if (speed_goal < myspeed) {
    //   myspeed -= increment;
    // }
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FL, speed_goal);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_FR, speed_goal);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RL, speed_goal);
    mgDrive_wheel.Speedclosedloopcontrol(MOT_CAN_ID_RR, speed_goal);
  } else {
    // myspeed = 0.0;
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FL);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_FR);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RL);
    mgDrive_wheel.Motor_stop(MOT_CAN_ID_RR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_FR);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RL);
    mgDrive_wheel.ReadMotorState2(MOT_CAN_ID_RR);
  }

  fd_doc[0] = speed_goal;
  fd_doc[1] = -1;
  fd_doc[2] = -1;
}
//^^^^^^^^^^^^^^^ MG_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^

// AK_Motor

//^^^^^^^^^^^^^^^ AK_Motor ^^^^^^^^^^^^^^^^^^^^^^^^^^