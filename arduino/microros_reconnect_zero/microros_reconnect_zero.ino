// Micro_ROS
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_srvs/srv/trigger.h>
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

// Micro_ROS
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

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_wait_set_t wait_set;

rcl_publisher_t pub1;
rcl_subscription_t sub;
rcl_service_t ser1;

std_msgs__msg__Float64 pub1_msg;
std_msgs__msg__Float64MultiArray command_msg;
std_srvs__srv__Trigger_Response soil_res;
std_srvs__srv__Trigger_Request soil_req;
static char res_message = '\0';

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    rcl_publish(&pub1, &pub1_msg, NULL);
    // msg.data++;
  }
}

void ros_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  pub1_msg.data = msg->data.data[0];
}

void service_callback(const void * req, void * res) {
  const std_srvs__srv__Trigger_Request * req_in = (const std_srvs__srv__Trigger_Request *)req;
  std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *)res;

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED

  res_in->success = true;
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
  RCCHECK(rclc_node_init_default(&node, "node_test", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &pub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "std_msgs_msg_Float64"));
  
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "command"));
  
  // create service
  RCCHECK(rclc_service_init_default(
    &ser1,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "/soil_trigger_service"));

  // Init the memory of your array in order to provide it to the executor.
  // If a message from ROS comes and it is bigger than this, it will be ignored, so ensure that capacities here are big enought.

  command_msg.data.capacity = 2;
  command_msg.data.size = 0;
  command_msg.data.data = (double *)malloc(command_msg.data.capacity * sizeof(double));

  command_msg.layout.dim.capacity = 2;
  command_msg.layout.dim.size = 0;
  command_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(command_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < command_msg.layout.dim.capacity; i++) {
    command_msg.layout.dim.data[i].label.capacity = 5;
    command_msg.layout.dim.data[i].label.size = 0;
    command_msg.layout.dim.data[i].label.data = (char *)malloc(command_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  soil_res.message.data = &res_message;
  soil_res.message.size = 1;
  soil_res.message.capacity = 1;  
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub, &command_msg, &ros_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &ser1, &soil_req, &soil_res, service_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub1, &node);
  rcl_subscription_fini(&sub, &node);
  rcl_service_fini(&ser1, &node);

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

  state = WAITING_AGENT;

  pub1_msg.data = 0;
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void loop() {
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

  // digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? 1 : 0);
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
}
