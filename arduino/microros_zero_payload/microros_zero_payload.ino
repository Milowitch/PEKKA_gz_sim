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
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_srvs/srv/trigger.h>
#include <rosidl_runtime_c/string_functions.h>
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^
#include <Arduino.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>

#define RS485_RE_PIN 12   3
#define RS485_DE_PIN 13   4
#define dirPin 8   // ขา DIR ของ Stepper Motor
#define stepPin 9  // ขา PUL ของ Stepper Motor

// Linear Actuator 1
#define IN1_1 7
#define IN2_1 6
#define ENA_1 10

// Linear Actuator 2
#define IN1_2 5
#define IN2_2 4
#define ENA_2 11

ModbusMaster modb1;

unsigned long currentMillis = millis();

const unsigned long retractDelay_Actuator1 = 5000;
const unsigned long retractDelay_Actuator2 = 5000;
const unsigned long holdTime_Actuator2 = 10000;

bool taskStarted = false;
bool stepperAtPosition1 = false;
bool actuator1Complete = false;
bool stepperAtPosition2 = false;
bool actuator2Complete = false;
int cycleCount = 0;
unsigned long previousMillis = 0;
unsigned long stateStartTime = 0;
int actuator_state = 0;  // ใช้ตัวแปร state กำหนดลำดับการทำงาน

// Stepper Motor settings
#define pitch 0.5               // ระยะที่ Linear Actuator เคลื่อนที่ได้ในหนึ่งรอบ
#define stepsPerRevolution 200  // จำนวนสเต็ปต่อหนึ่งรอบของ Stepper Motor

void preTransmission() {
  digitalWrite(RS485_DE_PIN, HIGH);
  digitalWrite(RS485_RE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_RE_PIN, LOW);
}

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

std_msgs__msg__String soil_msg;
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

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    uint8_t command[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08 };
    sendRawCommand(command, sizeof(command));

    SerialUSB.println("Testing Modbus Communication...");
    readAndPrintSensorData();
  }
}

void service_callback(const void *req, void *res) {
  const std_srvs__srv__Trigger_Request *req_in = (const std_srvs__srv__Trigger_Request *)req;
  std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *)res;

  if (!taskStarted) {
    SerialUSB.println("Start command received. Starting task...");
    resetState();  // รีเซ็ตค่าตัวแปรทุกตัว
    taskStarted = true;
    actuator_state = 1;  // เริ่มที่ state 1
    stateStartTime = millis();
  }

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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node_payload_system", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &pub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/soil/rawdata"));

  // create service
  RCCHECK(rclc_service_init_default(
    &ser1,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "/soil_trigger_service"));

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
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &ser1, &soil_req, &soil_res, service_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&pub1, &node);
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
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_RE_PIN, OUTPUT);

  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_RE_PIN, LOW);

  SerialUSB.begin(115200);
  Serial1.begin(4800);
  modb1.begin(0x01, Serial1);

  modb1.preTransmission(preTransmission);
  modb1.postTransmission(postTransmission);

  // Setup Stepper Motor
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  // Setup Linear Actuators
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(ENA_1, OUTPUT);

  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(ENA_2, OUTPUT);


  // Serial Monitor
  SerialUSB.println("Program initialized. Type 'Start' to begin.");
}

void loop() {
  currentMillis = millis();

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

  // ใส่ตรงนี้
  if (!taskStarted) {
    if (SerialUSB.available()) {
      String command = SerialUSB.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("Start")) {
        SerialUSB.println("Start command received. Starting task...");
        resetState();  // รีเซ็ตค่าตัวแปรทุกตัว
        taskStarted = true;
        actuator_state = 1;  // เริ่มที่ state 1
        stateStartTime = millis();
      } else {
        SerialUSB.println("Invalid command. Type 'Start' to begin.");
      }
    }
  }

  switch (actuator_state) {
    case 1:  // 1. Stepper Motor ไปจุดแรก (CW)
      SerialUSB.println("Moving Stepper Motor to Position 1 (CW)...");
      moveStepper(200, HIGH);
      stepperAtPosition1 = true;
      actuator_state = 2;
      stateStartTime = currentMillis;
      break;

    case 2:  // 2. Linear Actuator 1 ทำงาน
      if (stepperAtPosition1 && !actuator1Complete) {
        if (cycleCount < 3) {
          SerialUSB.print("Cycle: ");
          SerialUSB.println(cycleCount + 1);
          moveForwardActuator1();
          actuator_state = 3;
          stateStartTime = currentMillis;
        } else {
          actuator1Complete = true;
          SerialUSB.println("Actuator 1 complete!");
          actuator_state = 4;
          stateStartTime = currentMillis;
        }
      }
      break;

    case 3:  // รอ Retract Actuator 1
      if (currentMillis - stateStartTime >= retractDelay_Actuator1) {
        retractActuator1();
        actuator_state = 4;
        stateStartTime = currentMillis;
      }
      break;

    case 4:  // รอหยุด Actuator 1
      if (currentMillis - stateStartTime >= retractDelay_Actuator1) {
        stopActuator1();
        cycleCount++;
        if (cycleCount < 3) {
          actuator_state = 2;  // กลับไปทำซ้ำ
        } else {
          actuator1Complete = true;
          SerialUSB.println("Actuator 1 complete!");
          actuator_state = 5;
        }
        stateStartTime = currentMillis;
      }
      break;

    case 5:  // 3. Stepper Motor ไปจุดที่สอง (CCW)
      SerialUSB.println("Moving Stepper Motor to Position 2 (CCW)...");
      moveStepper(400, LOW);
      stepperAtPosition2 = true;
      actuator_state = 6;
      stateStartTime = currentMillis;
      break;

    case 6:  // 4. Linear Actuator 2 ทำงาน
      if (stepperAtPosition2 && !actuator2Complete) {
        SerialUSB.println("Actuator 2 starting...");
        moveForwardActuator2();
        actuator_state = 7;
        stateStartTime = currentMillis;
      }
      break;

    case 7:  // รอ Actuator 2 Retract
      if (currentMillis - stateStartTime >= retractDelay_Actuator2) {
        stopActuator2();
        SerialUSB.println("Holding position...");
        actuator_state = 8;
        stateStartTime = currentMillis;
      }
      break;

    case 8:  // รอ Hold Time
      if (currentMillis - stateStartTime >= holdTime_Actuator2) {
        retractActuator2();
        actuator_state = 9;
        stateStartTime = currentMillis;
        rcl_publish(&pub1, &soil_msg, NULL);
      }
      break;

    case 9:  // รอ Actuator 2 Retract
      if (currentMillis - stateStartTime >= retractDelay_Actuator2) {
        stopActuator2();
        actuator2Complete = true;
        SerialUSB.println("Actuator 2 complete!");
        actuator_state = 10;
        stateStartTime = currentMillis;
      }
      break;

    case 10:  // 5. กลับตำแหน่งเริ่มต้น
      SerialUSB.println("Resetting to initial position...");
      moveStepper(600, HIGH);
      SerialUSB.println("All tasks complete. Waiting for next 'Start' command...");
      taskStarted = false;
      actuator_state = 0;  // รีเซ็ตสถานะ
      break;
  }
}

// รีเซ็ตสถานะการทำงานทั้งหมด
void resetState() {
  cycleCount = 0;
  stepperAtPosition1 = false;
  actuator1Complete = false;
  stepperAtPosition2 = false;
  actuator2Complete = false;
}

void sendRawCommand(uint8_t *command, size_t length) {
  preTransmission();
  Serial1.write(command, length);
  Serial1.flush();
  postTransmission();

  // delay(50);  // Short delay to wait for response

  SerialUSB.println("Response from Slave: ");
  if (Serial1.available()) {
    uint8_t incomingByte = Serial1.read();
    SerialUSB.print("0x");
    SerialUSB.print(incomingByte, HEX);
    SerialUSB.print(" ");
  }
  SerialUSB.println();
}

void readAndPrintSensorData() {
  SerialUSB.print("Time: ");
  SerialUSB.println(millis());

  float moisture = readMoisture();
  SerialUSB.print("Moisture: ");
  SerialUSB.println(moisture);

  float temperature = readTemperature();
  SerialUSB.print("Temperature: ");
  SerialUSB.println(temperature);

  float conductivity = readConductivity();
  SerialUSB.print("Electrical Conductivity: ");
  SerialUSB.println(conductivity);

  float pH = readPH();
  SerialUSB.print("pH: ");
  SerialUSB.println(pH);

  float nitrogen = readNitrogen();
  SerialUSB.print("Nitrogen: ");
  SerialUSB.println(nitrogen);

  float phosphorus = readPhosphorus();
  SerialUSB.print("Phosphorus: ");
  SerialUSB.println(phosphorus);

  float potassium = readPotassium();
  SerialUSB.print("Potassium: ");
  SerialUSB.println(potassium);

  const int capacity = JSON_OBJECT_SIZE(7);
  StaticJsonDocument<capacity> jsonDoc;
  char char_msg[capacity];
  jsonDoc["misture"] = moisture;
  jsonDoc["tempe"] = temperature;
  jsonDoc["ec"] = conductivity;
  jsonDoc["ph"] = pH;
  jsonDoc["n"] = nitrogen;
  jsonDoc["p"] = phosphorus;
  jsonDoc["k"] = potassium;

  // แปลง JSON เป็น string
  serializeJson(jsonDoc, char_msg, sizeof(char_msg));
  // ตั้งค่า msg ด้วยข้อมูลใหม่
  rosidl_runtime_c__String__assign(&soil_msg.data, char_msg);
}

float readMoisture() {
  uint8_t result = modb1.readHoldingRegisters(0x0000, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) / 10.0 : 0.0;
}

float readTemperature() {
  uint8_t result = modb1.readHoldingRegisters(0x0001, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) / 10.0 : 0.0;
}

float readConductivity() {
  uint8_t result = modb1.readHoldingRegisters(0x0002, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) : 0.0;
}

float readPH() {
  uint8_t result = modb1.readHoldingRegisters(0x0003, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) / 10.0 : 0.0;
}

float readNitrogen() {
  uint8_t result = modb1.readHoldingRegisters(0x0004, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) : 0.0;
}

float readPhosphorus() {
  uint8_t result = modb1.readHoldingRegisters(0x0005, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) : 0.0;
}

float readPotassium() {
  uint8_t result = modb1.readHoldingRegisters(0x0006, 1);
  return (result == modb1.ku8MBSuccess) ? modb1.getResponseBuffer(0) : 0.0;
}

// ฟังก์ชัน Stepper Motor
void moveStepper(int steps, bool direction) {
  digitalWrite(dirPin, direction);

  int minDelay = 500;
  int maxDelay = 2000;
  int accelSteps = 50;

  for (int i = 0; i < steps; i++) {
    int delayTime;
    if (i < accelSteps) {
      delayTime = map(i, 0, accelSteps, maxDelay, minDelay);
    } else if (i >= steps - accelSteps) {
      delayTime = map(i, steps - accelSteps, steps, minDelay, maxDelay);
    } else {
      delayTime = minDelay;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
  }
}

// ฟังก์ชัน Linear Actuator 1
void moveForwardActuator1() {
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, 255);
}

void retractActuator1() {
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, 255);
}

void stopActuator1() {
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, 0);
}

// ฟังก์ชัน Linear Actuator 2
void moveForwardActuator2() {
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, 255);
}

void retractActuator2() {
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, 255);
}

void stopActuator2() {
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, 0);
}