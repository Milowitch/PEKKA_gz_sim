// battery
#include <ModbusMaster.h>

// Serial3 pin and pad definitions (in Arduino files Variant.h & modified Variant.cpp)
#define PIN_SERIAL3_RX (45ul)             // Pin description number for PIO_SERCOM on D5
#define PIN_SERIAL3_TX (44ul)             // Pin description number for PIO_SERCOM on D2
#define PAD_SERIAL3_TX (UART_TX_PAD_2)    // SERCOM pad 2
#define PAD_SERIAL3_RX (SERCOM_RX_PAD_3)  // SERCOM pad 3
// Instantiate the Serial3 class
Uart Serial3(&sercom2, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);

//#define PAD_SERIAL1_TX (UART_TX_PAD_0)    // SERCOM pad 2
//#define PAD_SERIAL1_RX (SERCOM_RX_PAD_0)  // SERCOM pad 3
//Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

// ตั้งค่า pin สำหรับต่อกับ MAX485
#define MAX485_RO1 1
#define MAX485_RE1 7
#define MAX485_DE1 6
#define MAX485_DI1 0

#define MAX485_RO2 2
#define MAX485_RE2 4
#define MAX485_DE2 3
#define MAX485_DI2 5

// Address ของ PZEM-017 : 0x01-0xF7
const uint8_t pzemSlaveAddr1 = 0x01;
const uint8_t pzemSlaveAddr2 = 0x02;

// ตั้งค่า shunt -->> 0x0000-100A, 0x0001-50A, 0x0002-200A, 0x0003-300A
const uint16_t NewshuntAddr = 0x0000;


const uint8_t SlaveParameter = 0x06;     /* Write command code to PZEM */
const uint16_t registerAddress = 0x0003; /* change shunt register address command code */

ModbusMaster mbm1;
ModbusMaster mbm2;

//unsigned long startMillis1;  // to count time during initial start up (PZEM Software got some error so need to have initial pending time)
//unsigned long startMillis2;

unsigned long previousMillis = 0;
const uint16_t interval = 1000;


float PZEMVoltage1, PZEMCurrent1, PZEMPower1, PZEMEnergy1;
float PZEMVoltage2, PZEMCurrent2, PZEMPower2, PZEMEnergy2;
// ^^^^^^^^^^^^^^   battery   ^^^^^^^^^^^^^^^^^

//#include "DFRobot_SHT20.h"
#include <M2M_LM75A.h>
#include <ArduinoJson.h>

#define LED_PIN A0
#define INT_RED_PIN 8
#define INT_GREEN_PIN 9
#define RELAY_1_PIN A1
#define RELAY_2_PIN A2

// Micro_ROS
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16.h>
#include <std_srvs/srv/trigger.h>

#include <rosidl_runtime_c/string_functions.h>
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
rcl_service_t ser1;

std_msgs__msg__String msg;

std_msgs__msg__Int16 wifi_msg;
std_srvs__srv__Trigger_Response fan_res;
std_srvs__srv__Trigger_Request fan_req;
static char res_message = '\0';

//DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
M2M_LM75A lm75a1(0x48);  // Address 0x48 (A0 connected to GND)
M2M_LM75A lm75a2(0x49);  // Address 0x49 (A0 connected to VCC)

float temp_lm75a1,temp_lm75a2;

// float status_wifi = 0;

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
    function_var();
    rcl_publish(&pub1, &msg, NULL);
  }
}

void subscription_callback(const void* msgin) {
  const std_msgs__msg__Int16* msg = (const std_msgs__msg__Int16*)msgin;
  // Serial.println("Received Float64MultiArray:");
  // for (size_t i = 0; i < msg->data.size; i++) {
  //   Serial.print("Data[");
  //   Serial.print(i);
  //   Serial.print("]: ");
  //   Serial.println(msg->data.data[i]);
  // }
  // status_wifi = msg->data.data[0];
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

bool fan = true;

void service_callback(const void* req, void* res) {
  const std_srvs__srv__Trigger_Request* req_in = (const std_srvs__srv__Trigger_Request*)req;
  std_srvs__srv__Trigger_Response* res_in = (std_srvs__srv__Trigger_Response*)res;

  fan = !fan;

  digitalWrite(RELAY_1_PIN, fan);
  digitalWrite(RELAY_2_PIN, fan);

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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node_health_system", "", &support));

  // create pub
  RCCHECK(rclc_publisher_init_default(
    &pub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "rover_health"));
  // create sub wifi
  RCCHECK(rclc_subscription_init_default(
    &sub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "wifi_status"));

  // create service
  RCCHECK(rclc_service_init_default(
    &ser1,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "/fan_trigger_service"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  fan_res.message.data = &res_message;
  fan_res.message.size = 1;
  fan_res.message.capacity = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub1, &wifi_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &ser1, &fan_req, &fan_res, service_callback));

  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub1, &node);
  rcl_subscription_fini(&sub1, &node);
  rcl_service_fini(&ser1, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
//^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

void function_var() {
  const int capacity = JSON_OBJECT_SIZE(10);
  StaticJsonDocument<capacity> jsonDoc;
  char char_msg[capacity];
  jsonDoc["temp1"] = temp_lm75a1;
  jsonDoc["temp2"] = temp_lm75a2;
  jsonDoc["voltage1"] = PZEMVoltage1;
  jsonDoc["current1"] = PZEMCurrent1;
  jsonDoc["power1"] = PZEMPower1;
  jsonDoc["energy1"] = PZEMEnergy1;
  jsonDoc["voltage2"] = PZEMVoltage2;
  jsonDoc["current2"] = PZEMCurrent2;
  jsonDoc["power2"] = PZEMPower2;
  jsonDoc["energy2"] = PZEMEnergy2;
  //SerialUSB.println("สวัสดี");

  // แปลง JSON เป็น string
  serializeJson(jsonDoc, char_msg, sizeof(char_msg));
  // ตั้งค่า msg ด้วยข้อมูลใหม่
  rosidl_runtime_c__String__assign(&msg.data, char_msg);
}

void setup() {
  // Micro_ROS
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

  SerialUSB.begin(115200);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(INT_RED_PIN, OUTPUT);
  digitalWrite(INT_RED_PIN, 1);
  digitalWrite(INT_GREEN_PIN, 1);
  pinMode(INT_GREEN_PIN, OUTPUT);
  //sht20.initSHT20();
  //sht20.checkSHT20();
  lm75a1.begin();
  lm75a2.begin();

  // battery
  //startMillis1 = millis();
  //startMillis2 = millis();
  Serial1.begin(9600);
  Serial3.begin(9600);

  pinMode(MAX485_RE1, OUTPUT); /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE1, OUTPUT); /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_RE2, OUTPUT); /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE2, OUTPUT); /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/

  digitalWrite(MAX485_RE1, 0); /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE1, 0); /* Arduino create output signal for pin DE as LOW (no output)*/
  digitalWrite(MAX485_RE2, 0); /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE2, 0); /* Arduino create output signal for pin DE as LOW (no output)*/

  mbm1.preTransmission(preTransmission1);  // Callbacks allow us to configure the RS485 transceiver correctly
  mbm1.postTransmission(postTransmission1);
  mbm2.preTransmission(preTransmission2);  // Callbacks allow us to configure the RS485 transceiver correctly
  mbm2.postTransmission(postTransmission2);

  mbm1.begin(pzemSlaveAddr1, Serial1);
  mbm2.begin(pzemSlaveAddr2, Serial3);
  delay(1000); /* after everything done, wait for 1 second */

  // รอครบ 5 วินาที แล้วตั้งค่า shunt และ address
 for(char i=0;i<10;i++){
    delay(500);
    SerialUSB.print(".");
  }
  SerialUSB.println(".");

  setShunt1(pzemSlaveAddr1);             // ตั้งค่า shunt
  changeAddress1(0xF8, pzemSlaveAddr1);  // ตั้งค่า address 0x01 ซื่งเป็นค่า default ของตัว PZEM-017
  // resetEnergy1(pzemSlaveAddr1);                      // รีเซ็ตค่า Energy[Wh] (หน่วยใช้ไฟสะสม)

  setShunt2(pzemSlaveAddr2);             // ตั้งค่า shunt
  changeAddress2(0xF8, pzemSlaveAddr2);  // ตั้งค่า address 0x01 ซื่งเป็นค่า default ของตัว PZEM-017
  // resetEnergy2(pzemSlaveAddr2);                      // รีเซ็ตค่า Energy[Wh] (หน่วยใช้ไฟสะสม)
  // ^^^^^^^^^^^^^^   battery   ^^^^^^^^^^^^^^^^^
}

bool LEDstatus = false;

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

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    LEDstatus = !LEDstatus;
    digitalWrite(LED_PIN, LEDstatus);
  }
  //^^^^^^^^^^^^^^^ Micro_ROS ^^^^^^^^^^^^^^^^^^^^^^^^^^

  // battery
  unsigned long currentMillis = millis();

  // จัดการโอเวอร์โฟลว์ของ millis()
  if (currentMillis < previousMillis) {
    previousMillis = 0;  // รีเซ็ตเวลาเมื่อโอเวอร์โฟลว์
  }

  // ถ้าเวลาผ่านไป สำหรับ loop1
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    SerialUSB.println(previousMillis);

    //uint8_t result1;                              /* Declare variable "result" as 8 bits */
    //result1 = mbm1.readInputRegisters(0x0000, 6); /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
    if (mbm1.readInputRegisters(0x0000, 6) == mbm1.ku8MBSuccess)             /* If there is a response */
    {
      uint32_t tempdouble = 0x00000000;                      /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
      PZEMVoltage1 = mbm1.getResponseBuffer(0x0000) / 100.0; /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
      // 0x0000 to 0x0008 are the register address of the measurement value
      PZEMCurrent1 = mbm1.getResponseBuffer(0x0001) / 100.0; /* get the 16bit value for the current value, divide it by 100 (as per manual) */

      tempdouble = (mbm1.getResponseBuffer(0x0003) << 16) + mbm1.getResponseBuffer(0x0002); /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMPower1 = tempdouble / 10.0;                                                       /* Divide the value by 10 to get actual power value (as per manual) */

      tempdouble = (mbm1.getResponseBuffer(0x0005) << 16) + mbm1.getResponseBuffer(0x0004); /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMEnergy1 = tempdouble;
    } else  // ถ้าติดต่อ PZEM-017 ไม่ได้ ให้ใส่ค่า NAN(Not a Number)
    {
      PZEMVoltage1 = NAN;
      PZEMCurrent1 = NAN;
      PZEMPower1 = NAN;
      PZEMEnergy1 = NAN;
    }

    //uint8_t result2;                              /* Declare variable "result" as 8 bits */
    //result2 = mbm2.readInputRegisters(0x0000, 6); /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
    if (mbm2.readInputRegisters(0x0000, 6) == mbm2.ku8MBSuccess)             /* If there is a response */
    {
      uint32_t tempdouble = 0x00000000;                      /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
      PZEMVoltage2 = mbm2.getResponseBuffer(0x0000) / 100.0; /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
      // 0x0000 to 0x0008 are the register address of the measurement value
      PZEMCurrent2 = mbm2.getResponseBuffer(0x0001) / 100.0; /* get the 16bit value for the current value, divide it by 100 (as per manual) */

      tempdouble = (mbm2.getResponseBuffer(0x0003) << 16) + mbm2.getResponseBuffer(0x0002); /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMPower2 = tempdouble / 10.0;                                                       /* Divide the value by 10 to get actual power value (as per manual) */

      tempdouble = (mbm2.getResponseBuffer(0x0005) << 16) + mbm2.getResponseBuffer(0x0004); /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMEnergy2 = tempdouble;
    } else  // ถ้าติดต่อ PZEM-017 ไม่ได้ ให้ใส่ค่า NAN(Not a Number)
    {
      PZEMVoltage2 = NAN;
      PZEMCurrent2 = NAN;
      PZEMPower2 = NAN;
      PZEMEnergy2 = NAN;
    }

    temp_lm75a1 = lm75a1.getTemperature();
    temp_lm75a2 = lm75a2.getTemperature();

    // แสดงค่าที่ได้จากบน Serial monitor
    SerialUSB.print("Vdc ");
    SerialUSB.print(pzemSlaveAddr1);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMVoltage1);
    SerialUSB.println(" V ");
    SerialUSB.print("Idc : ");
    SerialUSB.print(pzemSlaveAddr1);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMCurrent1);
    SerialUSB.println(" A ");
    SerialUSB.print("Power : ");
    SerialUSB.print(pzemSlaveAddr1);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMPower1);
    SerialUSB.println(" W ");
    SerialUSB.print("Energy : ");
    SerialUSB.print(pzemSlaveAddr1);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMEnergy1);
    SerialUSB.println(" Wh ");


    // แสดงค่าที่ได้จากบน Serial monitor
    SerialUSB.print("Vdc ");
    SerialUSB.print(pzemSlaveAddr2);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMVoltage2);
    SerialUSB.println(" V ");
    SerialUSB.print("Idc : ");
    SerialUSB.print(pzemSlaveAddr2);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMCurrent2);
    SerialUSB.println(" A ");
    SerialUSB.print("Power : ");
    SerialUSB.print(pzemSlaveAddr2);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMPower2);
    SerialUSB.println(" W ");
    SerialUSB.print("Energy : ");
    SerialUSB.print(pzemSlaveAddr2);
    SerialUSB.print(" : ");
    SerialUSB.print(PZEMEnergy2);
    SerialUSB.println(" Wh ");

    
    SerialUSB.print("Temp 1: ");
    SerialUSB.println(temp_lm75a1);
    SerialUSB.print("Temp 2: ");
    SerialUSB.println(temp_lm75a2);
  }
  // ^^^^^^^^^^^^^^   battery   ^^^^^^^^^^^^^^^^^
}

// battery
void preTransmission1() /* transmission program when triggered*/
{
  digitalWrite(MAX485_RE1, 1); /* put RE Pin to high*/
  digitalWrite(MAX485_DE1, 1); /* put DE Pin to high*/
}

void postTransmission1()       /* Reception program when triggered*/
{                              // When both RE and DE Pin are low, converter is allow to receive communication
  digitalWrite(MAX485_RE1, 0); /* put RE Pin to low*/
  digitalWrite(MAX485_DE1, 0); /* put DE Pin to low*/
}

void setShunt1(uint8_t slaveAddr)  //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  //static uint8_t SlaveParameter = 0x06;     /* Write command code to PZEM */
  //static uint16_t registerAddress = 0x0003; /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                  /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);  // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission1(); /* trigger transmission mode*/

  Serial1.write(slaveAddr); /* these whole process code sequence refer to manual*/
  Serial1.write(SlaveParameter);
  Serial1.write(highByte(registerAddress));
  Serial1.write(lowByte(registerAddress));
  Serial1.write(highByte(NewshuntAddr));
  Serial1.write(lowByte(NewshuntAddr));
  Serial1.write(lowByte(u16CRC));
  Serial1.write(highByte(u16CRC));
  // delay(10);
  postTransmission1(); /* trigger reception mode*/
  // delay(100);
}

void resetEnergy1(uint8_t pzemSlaveAddr)  // reset energy for Meter 1
{
  uint16_t u16CRC = 0xFFFF;           /* declare CRC check 16 bits*/
  static uint8_t resetCommand = 0x42; /* reset command code*/
  uint8_t slaveAddr = pzemSlaveAddr;  // if you set different address, make sure this slaveAddr must change also
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission1();              /* trigger transmission mode*/
  Serial1.write(slaveAddr);        /* send device address in 8 bit*/
  Serial1.write(resetCommand);     /* send reset command */
  Serial1.write(lowByte(u16CRC));  /* send CRC check code low byte  (1st part) */
  Serial1.write(highByte(u16CRC)); /* send CRC check code high byte (2nd part) */
  // delay(10);
  postTransmission1(); /* trigger reception mode*/
  // delay(100);
}

void changeAddress1(uint8_t OldslaveAddr, uint8_t NewslaveAddr)  //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  //static uint8_t SlaveParameter = 0x06;         /* Write command code to PZEM */
  //static uint16_t registerAddress = 0x0002;     /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                     /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr);  // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  preTransmission1();          /* trigger transmission mode*/
  Serial1.write(OldslaveAddr); /* these whole process code sequence refer to manual*/
  Serial1.write(SlaveParameter);
  Serial1.write(highByte(registerAddress));
  Serial1.write(lowByte(registerAddress));
  Serial1.write(highByte(NewslaveAddr));
  Serial1.write(lowByte(NewslaveAddr));
  Serial1.write(lowByte(u16CRC));
  Serial1.write(highByte(u16CRC));
  // delay(10);
  postTransmission1(); /* trigger reception mode*/
  // delay(100);
}

void preTransmission2() /* transmission program when triggered*/
{
  digitalWrite(MAX485_RE2, 1); /* put RE Pin to high*/
  digitalWrite(MAX485_DE2, 1); /* put DE Pin to high*/
}

void postTransmission2()       /* Reception program when triggered*/
{                              // When both RE and DE Pin are low, converter is allow to receive communication
  digitalWrite(MAX485_RE2, 0); /* put RE Pin to low*/
  digitalWrite(MAX485_DE2, 0); /* put DE Pin to low*/
}

void setShunt2(uint8_t slaveAddr)  //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  //static uint8_t SlaveParameter = 0x06;     /* Write command code to PZEM */
  //static uint16_t registerAddress = 0x0003; /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                  /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);  // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission2(); /* trigger transmission mode*/

  Serial3.write(slaveAddr); /* these whole process code sequence refer to manual*/
  Serial3.write(SlaveParameter);
  Serial3.write(highByte(registerAddress));
  Serial3.write(lowByte(registerAddress));
  Serial3.write(highByte(NewshuntAddr));
  Serial3.write(lowByte(NewshuntAddr));
  Serial3.write(lowByte(u16CRC));
  Serial3.write(highByte(u16CRC));
  // delay(10);
  postTransmission2(); /* trigger reception mode*/
  // delay(100);
}

void resetEnergy2(uint8_t pzemSlaveAddr)  // reset energy for Meter 1
{
  uint16_t u16CRC = 0xFFFF;           /* declare CRC check 16 bits*/
  static uint8_t resetCommand = 0x42; /* reset command code*/
  uint8_t slaveAddr = pzemSlaveAddr;  // if you set different address, make sure this slaveAddr must change also
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission2();              /* trigger transmission mode*/
  Serial3.write(slaveAddr);        /* send device address in 8 bit*/
  Serial3.write(resetCommand);     /* send reset command */
  Serial3.write(lowByte(u16CRC));  /* send CRC check code low byte  (1st part) */
  Serial3.write(highByte(u16CRC)); /* send CRC check code high byte (2nd part) */
  // delay(10);
  postTransmission2(); /* trigger reception mode*/
  // delay(100);
}

void changeAddress2(uint8_t OldslaveAddr, uint8_t NewslaveAddr)  //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  //static uint8_t SlaveParameter = 0x06;         /* Write command code to PZEM */
  //static uint16_t registerAddress = 0x0002;     /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                     /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr);  // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  preTransmission2();          /* trigger transmission mode*/
  Serial3.write(OldslaveAddr); /* these whole process code sequence refer to manual*/
  Serial3.write(SlaveParameter);
  Serial3.write(highByte(registerAddress));
  Serial3.write(lowByte(registerAddress));
  Serial3.write(highByte(NewslaveAddr));
  Serial3.write(lowByte(NewslaveAddr));
  Serial3.write(lowByte(u16CRC));
  Serial3.write(highByte(u16CRC));
  // delay(10);
  postTransmission2(); /* trigger reception mode*/
  // delay(100);
}

void SERCOM2_Handler()  // Interrupt handler for SERCOM2
{
  Serial3.IrqHandler();
}
/*
void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}
*/
// ^^^^^^^^^^^^^^   battery   ^^^^^^^^^^^^^^^^^