// Temps
#include <Wire.h>
#include "wiring_private.h"
#include "DFRobot_SHT20.h"
#include <ArduinoJson.h>
//^^^^^^^^^^^^ Temps ^^^^^^^^^^^^^^^^^^^^^^^
// Micro_ROS
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <rosidl_runtime_c/string_functions.h>
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

// Temps
// TwoWire myWire(&sercom1, 11, 13);

DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
// DFRobot_SHT20 sht20[2] = {
//   DFRobot_SHT20(&Wire, SHT20_I2C_ADDR),
//   DFRobot_SHT20(&myWire, SHT20_I2C_ADDR),
// };
#define INT_RED_PIN 2
#define INT_GREEN_PIN 4
//^^^^^^^^^^^^ Temps ^^^^^^^^^^^^^^^^^^^^^^^

// Micro_ROS
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

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t pub1;
rcl_subscription_t sub1;
bool micro_ros_init_successful;

std_msgs__msg__String temp_msg;
std_msgs__msg__Int8 msg;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

const char* stateStrings[] = {
  "WAITING_AGENT",
  "AGENT_AVAILABLE",
  "AGENT_CONNECTED",
  "AGENT_DISCONNECTED"
};

const int capacity = JSON_ARRAY_SIZE(2) + 2 * JSON_OBJECT_SIZE(3);
StaticJsonDocument<capacity> doc;
char char_temp_msg[capacity];

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    rosidl_runtime_c__String__assign(&temp_msg.data, char_temp_msg);
    rcl_publish(&pub1, &temp_msg, NULL);
  }
}

void robot_health_callback(const void* msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;

  if (msg->data == 0) {
    digitalWrite(INT_RED_PIN, 0);
    digitalWrite(INT_GREEN_PIN, 1);
  } else if (msg->data == 1) {
    digitalWrite(INT_RED_PIN, 0);
    digitalWrite(INT_GREEN_PIN, 0);
  } else if (msg->data == 2) {
    digitalWrite(INT_RED_PIN, 1);
    digitalWrite(INT_GREEN_PIN, 0);
  } else {
    digitalWrite(INT_RED_PIN, 1);
    digitalWrite(INT_GREEN_PIN, 1);
  }
}
// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node_robot_health_system", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &pub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "temp/rawdata"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &sub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/rover_wifi_status"));

  // create timer,
  timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub1, &msg, &robot_health_callback, ALWAYS));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  std_msgs__msg__String__init(&temp_msg);

  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub1, &node);
  rcl_subscription_fini(&sub1, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {
  // SerialUSB.begin(115200);

  // Temps
  sht20.initSHT20();

  // sht20[0].initSHT20();
  // sht20[1].initSHT20();

  // myWire.begin();

  // pinPeripheral(11, PIO_SERCOM);
  // pinPeripheral(13, PIO_SERCOM);

  // sht20[0].checkSHT20();
  delay(100);
  sht20.checkSHT20();
  //^^^^^^^^^^^^ Temps ^^^^^^^^^^^^^^^^^^^^^^^

  // Micro_ROS
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  pinMode(INT_RED_PIN, OUTPUT);
  pinMode(INT_GREEN_PIN, OUTPUT);
  digitalWrite(INT_RED_PIN, 1);
  digitalWrite(INT_GREEN_PIN, 1);

  state = WAITING_AGENT;

  // temp_msg.data = "None";
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}

unsigned long currentmilli = millis();
unsigned long premilli[3] = { 0, 0, 0 };
bool LEDstatus = false;

void loop() {
  currentmilli = millis();

  // Micro_ROS
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
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
    if (currentmilli - premilli[2] >= 500) {
      premilli[2] = currentmilli;

      LEDstatus = !LEDstatus;
      digitalWrite(LED_PIN, LEDstatus);
    }

    digitalWrite(INT_RED_PIN, 1);
    digitalWrite(INT_GREEN_PIN, 1);
  }

  if (currentmilli - premilli[0] >= 5000) {
    premilli[0] = currentmilli;

    // SerialUSB.println(stateStrings[state]);

    doc[0]["name"] = "Temp_0";
    doc[0]["temp"] = String(sht20.readTemperature(), 2);  // Read Temperature
    doc[0]["humd"] = String(sht20.readHumidity(), 2);     // Read Humidity
  }
  if (currentmilli - premilli[1] >= 5100) {
    premilli[1] = currentmilli;

    doc[1]["name"] = "Temp_1";
    doc[1]["temp"] = String(sht20.readTemperature(), 2);  // Read Temperature
    doc[1]["humd"] = String(sht20.readHumidity(), 2);     // Read Humidity

    serializeJson(doc, char_temp_msg, sizeof(char_temp_msg));
    // SerialUSB.println(char_temp_msg);
  }
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}
