// GPS IMU
#include <REG.h>
#include <wit_c_sdk.h>
#include <TinyGPS++.h>

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

HardwareSerial neogps(1);
HardwareSerial imu(2);

TinyGPSPlus gps;

#define TX1pin 2
#define RX1pin 4
#define TX2pin 17
#define RX2pin 16
//^^^^^^^^^^^^^^^^^^^ GPS IMU ^^^^^^^^^^^^^^^^^^^^^^^^
// ROS2
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__NavSatFix gps_msg;

rcl_publisher_t pub1;
rcl_publisher_t pub2;
// rcl_publisher_t pub3;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
bool micro_ros_init_successful;
struct timespec ts;
extern int clock_gettime(clockid_t unused, struct timespec *tp);

#define LED_PIN 2
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
    rcl_publish(&pub1, &imu_msg, NULL);
    rcl_publish(&pub2, &gps_msg, NULL);
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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_imu_gps_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &pub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/raw"));
  RCCHECK(rclc_publisher_init_best_effort(
    &pub2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
    "gps/raw"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub1, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
//^^^^^^^^^^^^^^^^^^^ ROS2 ^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;
  // put your setup code here, to run once:
  Serial.begin(115200);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  // Serial.print("\r\n********************** wit-motion normal example  ************************\r\n");
  AutoScanSensor();
  neogps.begin(9600, SERIAL_8N1, RX1pin, TX1pin);

  imu_msg.header.frame_id.data = "witmoimu";
  imu_msg.header.frame_id.size = 10;
  gps_msg.header.frame_id.data = "neogps";
  gps_msg.header.frame_id.size = 10;
}

void loop() {
  clock_gettime(CLOCK_REALTIME, &ts);
  static float fAcc[3], fGyro[3], fAngle[3];
  while (imu.available()) {
    WitSerialDataIn(imu.read());
  }
  // while (Serial.available()) {
  //   CopeCmdData(Serial.read());
  // }
  CmdProcess();
  if (s_cDataUpdate) {
    for (int i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
    }
    if (s_cDataUpdate & ACC_UPDATE) {
      // Serial.print("acc:");
      // Serial.print(fAcc[0], 3);
      // Serial.print(" ");
      // Serial.print(fAcc[1], 3);
      // Serial.print(" ");
      // Serial.print(fAcc[2], 3);
      // Serial.print("\r\n");
      imu_msg.linear_acceleration.x = fAcc[0];
      imu_msg.linear_acceleration.y = fAcc[1];
      imu_msg.linear_acceleration.z = fAcc[2];
      s_cDataUpdate &= ~ACC_UPDATE;
    }
    if (s_cDataUpdate & GYRO_UPDATE) {
      // Serial.print("gyro:");
      // Serial.print(fGyro[0], 1);
      // Serial.print(" ");
      // Serial.print(fGyro[1], 1);
      // Serial.print(" ");
      // Serial.print(fGyro[2], 1);
      // Serial.print("\r\n");
      imu_msg.angular_velocity.x = fGyro[0];
      imu_msg.angular_velocity.y = fGyro[1];
      imu_msg.angular_velocity.z = fGyro[2];
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if (s_cDataUpdate & ANGLE_UPDATE) {
      // Serial.print("angle:");
      // Serial.print(fAngle[0], 3);
      // Serial.print(" ");
      // Serial.print(fAngle[1], 3);
      // Serial.print(" ");
      // Serial.print(fAngle[2], 3);
      // Serial.print("\r\n");
      imu_msg.orientation.x = fAngle[0];
      imu_msg.orientation.y = fAngle[1];
      imu_msg.orientation.z = fAngle[2];
      imu_msg.orientation.w = 0.0;
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    if (s_cDataUpdate & MAG_UPDATE) {
      // Serial.print("mag:");
      // Serial.print(sReg[HX]);
      // Serial.print(" ");
      // Serial.print(sReg[HY]);
      // Serial.print(" ");
      // Serial.print(sReg[HZ]);
      // Serial.print("\r\n");
      s_cDataUpdate &= ~MAG_UPDATE;
    }
    imu_msg.header.stamp.sec = ts.tv_sec;
    imu_msg.header.stamp.nanosec = ts.tv_nsec;
    s_cDataUpdate = 0;
  }

  // GPS
  static boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    if (neogps.available()) {
      // Serial.write(neogps.read());
      if (gps.encode(neogps.read())) {
        newData = true;
      }
    }
  }

  //If newData is true
  if (newData == true) {
    newData = false;
    // Serial.print("Satellites ");
    // Serial.println(gps.satellites.value());
    // Serial.print("Lat ");
    // Serial.println(gps.location.lat(), 5);
    // Serial.print("Lng ");
    // Serial.println(gps.location.lng(), 5);
    // Serial.print("Speed KMPH ");
    // Serial.println(gps.speed.kmph());
    //print_speed();
    gps_msg.header.stamp.sec = ts.tv_sec;
    gps_msg.header.stamp.nanosec = ts.tv_nsec;
    gps_msg.latitude = gps.location.lat();
    gps_msg.longitude = gps.location.lng();
    gps_msg.altitude = gps.altitude.value();
  } else {
  }
  //^^^^^^^^^^^^^^  GPS  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // ROS2
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(600, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(600, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(600));
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
  //^^^^^^^^^^^^^^^^^^ ROS2 ^^^^^^^^^^^^^^^^^^^^^
  // delay(1);
}

void CopeCmdData(unsigned char ucData) {
  static unsigned char s_ucData[50], s_ucRxCnt = 0;

  s_ucData[s_ucRxCnt++] = ucData;
  if (s_ucRxCnt < 3) return;  //Less than three data returned
  if (s_ucRxCnt >= 50) s_ucRxCnt = 0;
  if (s_ucRxCnt >= 3) {
    if ((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
      s_cCmd = s_ucData[0];
      memset(s_ucData, 0, 50);
      s_ucRxCnt = 0;
    } else {
      s_ucData[0] = s_ucData[1];
      s_ucData[1] = s_ucData[2];
      s_ucRxCnt = 2;
    }
  }
}
static void ShowHelp(void) {
  // Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
  // Serial.print("\r\n************************          HELP           ************************\r\n");
  // Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
  // Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  // Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
  // Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
  // Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
  // Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
  // Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  // Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  // Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  // Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  // Serial.print("UART SEND:h\\r\\n   help.\r\n");
  // Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void) {
  switch (s_cCmd) {
    case 'a':
      // if (WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
      break;
    case 'm':
      // if (WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'e':
      // if (WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'u':
      // if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'U':
      // if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'B':
      // if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      // else {
      imu.begin(c_uiBaud[WIT_BAUD_115200], SERIAL_8N1, RX2pin, TX2pin);
      // Serial.print(" 115200 Baud rate modified successfully\r\n");
      // }
      break;
    case 'b':
      // if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      // else {
      imu.begin(c_uiBaud[WIT_BAUD_9600], SERIAL_8N1, RX2pin, TX2pin);
      // Serial.print(" 9600 Baud rate modified successfully\r\n");
      // }
      break;
    case 'r':
      // if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      // else Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'R':
      if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      // else Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'C':
      // if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c':
      // if (WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'h':
      ShowHelp();
      break;
    default: break;
  }
  s_cCmd = 0xff;
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  imu.write(p_data, uiSize);
  imu.flush();
}
static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  static int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

static void AutoScanSensor(void) {
  static int i, iRetry;

  for (i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    imu.begin(c_uiBaud[i], SERIAL_8N1, RX2pin, TX2pin);
    imu.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (imu.available()) {
        WitSerialDataIn(imu.read());
      }
      if (s_cDataUpdate != 0) {
        // Serial.print(c_uiBaud[i]);
        // Serial.print(" baud find sensor\r\n\r\n");
        ShowHelp();
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  // Serial.print("can not find sensor\r\n");
  // Serial.print("please check your connection\r\n");
}