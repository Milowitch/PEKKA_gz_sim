#include <micro_ros_arduino.h>
#include <std_srvs/srv/trigger.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;
rcl_wait_set_t wait_set;

std_srvs__srv__Trigger_Response res;
std_srvs__srv__Trigger_Request req;
static char res_message = '\0';

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void service_callback(const void * req, void * res) {
  const std_srvs__srv__Trigger_Request * req_in = (const std_srvs__srv__Trigger_Request *) req;
  std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;

  // เปิดหรือปิด LED ตามการกระทำที่คุณต้องการ
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED

  res_in->success = true;
}

void setup() {
  set_microros_transports();
  delay(1000); 

  allocator = rcl_get_default_allocator();

  // initialize micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "trigger_led_server", "", &support));

  // initialize the service with std_srvs/srv/Trigger
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/trigger_led"));

  // initialize executor
  res.message.data = &res_message;
  res.message.size = 1;
  res.message.capacity = 1;  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
  
  // setup the LED pin
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
