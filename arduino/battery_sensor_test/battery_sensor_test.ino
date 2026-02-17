/*
    PZEM-017 DC Power Meter

    Note :  โปรดระมัดระวังอันตรายจากการต่อไฟฟ้า และอุปกรณ์ที่อาจเสียหายจากการต่อใช้งาน ทางเราไม่รับผิดชอบในความเสียหาย สูญเสีย ไม่ว่าทางตรงหรือทางอ้อม หรือค่าใช้จ่ายใดๆ
    Note :  โค๊ดต้นฉบับตัวอย่างจากเว็บนี้ : https://solarduino.com/pzem-017-dc-energy-meter-online-monitoring-with-blynk-app/
    Note :  และนำมาดัดแปลงโดย https://www.IoTbundle.com

-----------------------------------------------------------------------------------------------------------------------------------------------
    การใช้งาน
    โค๊ดนี้จะต่อใช้งานบอร์ด Wemos(ESP8266) กับ เซนเซอร์วัดไฟฟ้ากระแสตรง PZEM-017 โดยสื่อสารกับด้วยโมดูล UART TTL to RS485(MAX485) ชนิด 4 พิน (DI DE Re & RO)
    ------------------      --------------------------------------      ------------
    | wemos(ESP8266) |  ->  | UART TTL to RS485 converter module |  ->  | PZEM-017 |
    ------------------      --------------------------------------      ------------
-----------------------------------------------------------------------------------------------------------------------------------------------
    การต่อสาย
    Wemos   ->   MAX485  ->   PZEM-017

    D0             DI
    D5             DE
    D6             RE
    D7             RO

    5v            VCC            5V
    GND           GND            GND
                   A             A
                   B             B
-----------------------------------------------------------------------------------------------------------------------------------------------
    การแก้ไขโค๊ด
    - ติดตั้ง ESP8266 จาก Board manager
    - ติดตั้ง Library "ModbusMaster" จาก Library manager
    - ตั้งค่า Shunt ให้ถูกรุ่นที่ตัวแปร "NewshuntAddr"
-----------------------------------------------------------------------------------------------------------------------------------------------
*/


/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/////////////*/

#include <ModbusMaster.h>

// Serial3 pin and pad definitions (in Arduino files Variant.h & modified Variant.cpp)
#define PIN_SERIAL3_RX       (45ul)               // Pin description number for PIO_SERCOM on D5
#define PIN_SERIAL3_TX       (44ul)               // Pin description number for PIO_SERCOM on D2
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3
// Instantiate the Serial3 class
Uart Serial3(&sercom2, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);

// ตั้งค่า pin สำหรับต่อกับ MAX485
#define MAX485_RO1  1 // Tx
#define MAX485_RE1  7
#define MAX485_DE1  6
#define MAX485_DI1  0 // Rx

#define MAX485_RO2  2 // Tx
#define MAX485_RE2  4
#define MAX485_DE2  3
#define MAX485_DI2  5 // Rx

// Address ของ PZEM-017 : 0x01-0xF7
static uint8_t pzemSlaveAddr1 = 0x01;
static uint8_t pzemSlaveAddr2 = 0x02;

// ตั้งค่า shunt -->> 0x0000-100A, 0x0001-50A, 0x0002-200A, 0x0003-300A
static uint16_t NewshuntAddr = 0x0000;

ModbusMaster mbm1;
ModbusMaster mbm2;

unsigned long startMillis1;                           // to count time during initial start up (PZEM Software got some error so need to have initial pending time)
unsigned long startMillis2;

unsigned long previousMillis = 0;
const long interval = 500;
unsigned long currentMillis = 0;

float PZEMVoltage1, PZEMCurrent1, PZEMPower1, PZEMEnergy1;
float PZEMVoltage2, PZEMCurrent2, PZEMPower2, PZEMEnergy2;

void setup()
{
  startMillis1 = millis();
  startMillis2 = millis();
  SerialUSB.begin(115200);
  Serial1.begin(9600);
  Serial3.begin(9600);

  pinMode(MAX485_RE1, OUTPUT);                           /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE1, OUTPUT);                           /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_RE2, OUTPUT);                           /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE2, OUTPUT);                           /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/

  digitalWrite(MAX485_RE1, 0);                           /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE1, 0);                           /* Arduino create output signal for pin DE as LOW (no output)*/
  digitalWrite(MAX485_RE2, 0);                           /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE2, 0);                           /* Arduino create output signal for pin DE as LOW (no output)*/

  mbm1.preTransmission(preTransmission1);                // Callbacks allow us to configure the RS485 transceiver correctly
  mbm1.postTransmission(postTransmission1);
  mbm2.preTransmission(preTransmission2);                // Callbacks allow us to configure the RS485 transceiver correctly
  mbm2.postTransmission(postTransmission2);

  mbm1.begin(pzemSlaveAddr1, Serial1);
  mbm2.begin(pzemSlaveAddr2, Serial3);
  delay(1000);                                          /* after everything done, wait for 1 second */

  // รอครบ 5 วินาที แล้วตั้งค่า shunt และ address
  while (millis() - startMillis1 < 5000) {
    delay(500);
    SerialUSB.print(".");
  }
  SerialUSB.println(".");

  setShunt1(pzemSlaveAddr1);                   // ตั้งค่า shunt
  changeAddress1(0xF8, pzemSlaveAddr1);                 // ตั้งค่า address 0x01 ซื่งเป็นค่า default ของตัว PZEM-017
  // resetEnergy1(pzemSlaveAddr1);                      // รีเซ็ตค่า Energy[Wh] (หน่วยใช้ไฟสะสม)

  setShunt2(pzemSlaveAddr2);                   // ตั้งค่า shunt
  changeAddress2(0xF8, pzemSlaveAddr2);                 // ตั้งค่า address 0x01 ซื่งเป็นค่า default ของตัว PZEM-017
  // resetEnergy2(pzemSlaveAddr2);                      // รีเซ็ตค่า Energy[Wh] (หน่วยใช้ไฟสะสม)
}

void loop()
{
  currentMillis = millis();

  // จัดการโอเวอร์โฟลว์ของ millis()
  if (currentMillis < previousMillis) {
    previousMillis = 0;  // รีเซ็ตเวลาเมื่อโอเวอร์โฟลว์
  }

  // ถ้าเวลาผ่านไป สำหรับ loop1
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    uint8_t result1;                                                                                 /* Declare variable "result" as 8 bits */
    result1 = mbm1.readInputRegisters(0x0000, 6);                                                    /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
    if (result1 == mbm1.ku8MBSuccess)                                                                /* If there is a response */
    {
      uint32_t tempdouble = 0x00000000;                                                           /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
      PZEMVoltage1 = mbm1.getResponseBuffer(0x0000) / 100.0;                                       /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
      // 0x0000 to 0x0008 are the register address of the measurement value
      PZEMCurrent1 = mbm1.getResponseBuffer(0x0001) / 100.0;                                       /* get the 16bit value for the current value, divide it by 100 (as per manual) */

      tempdouble =  (mbm1.getResponseBuffer(0x0003) << 16) + mbm1.getResponseBuffer(0x0002);      /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMPower1 = tempdouble / 10.0;                                                              /* Divide the value by 10 to get actual power value (as per manual) */

      tempdouble =  (mbm1.getResponseBuffer(0x0005) << 16) + mbm1.getResponseBuffer(0x0004);      /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMEnergy1 = tempdouble;
    }
    else // ถ้าติดต่อ PZEM-017 ไม่ได้ ให้ใส่ค่า NAN(Not a Number)
    {
      PZEMVoltage1 = NAN;
      PZEMCurrent1 = NAN;
      PZEMPower1 = NAN;
      PZEMEnergy1 = NAN;
    }

    // แสดงค่าที่ได้จากบน Serial monitor
    SerialUSB.print("Vdc "); SerialUSB.print(pzemSlaveAddr1); SerialUSB.print(" : "); SerialUSB.print(PZEMVoltage1); SerialUSB.println(" V ");
    SerialUSB.print("Idc : "); SerialUSB.print(pzemSlaveAddr1); SerialUSB.print(" : "); SerialUSB.print(PZEMCurrent1); SerialUSB.println(" A ");
    SerialUSB.print("Power : "); SerialUSB.print(pzemSlaveAddr1); SerialUSB.print(" : "); SerialUSB.print(PZEMPower1); SerialUSB.println(" W ");
    SerialUSB.print("Energy : "); SerialUSB.print(pzemSlaveAddr1); SerialUSB.print(" : "); SerialUSB.print(PZEMEnergy1); SerialUSB.println(" Wh ");

    uint8_t result2;                                                                                 /* Declare variable "result" as 8 bits */
    result2 = mbm2.readInputRegisters(0x0000, 6);                                                    /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
    if (result2 == mbm2.ku8MBSuccess)                                                                /* If there is a response */
    {
        uint32_t tempdouble = 0x00000000;                                                           /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
        PZEMVoltage2 = mbm2.getResponseBuffer(0x0000) / 100.0;                                       /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
        // 0x0000 to 0x0008 are the register address of the measurement value
        PZEMCurrent2 = mbm2.getResponseBuffer(0x0001) / 100.0;                                       /* get the 16bit value for the current value, divide it by 100 (as per manual) */

        tempdouble =  (mbm2.getResponseBuffer(0x0003) << 16) + mbm2.getResponseBuffer(0x0002);      /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
        PZEMPower2 = tempdouble / 10.0;                                                              /* Divide the value by 10 to get actual power value (as per manual) */

        tempdouble =  (mbm2.getResponseBuffer(0x0005) << 16) + mbm2.getResponseBuffer(0x0004);      /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
        PZEMEnergy2 = tempdouble;
    }
    else // ถ้าติดต่อ PZEM-017 ไม่ได้ ให้ใส่ค่า NAN(Not a Number)
    {
      PZEMVoltage2 = NAN;
      PZEMCurrent2 = NAN;
      PZEMPower2 = NAN;
      PZEMEnergy2 = NAN;
    }
    // แสดงค่าที่ได้จากบน Serial monitor
    SerialUSB.print("Vdc "); SerialUSB.print(pzemSlaveAddr2); SerialUSB.print(" : "); SerialUSB.print(PZEMVoltage2); SerialUSB.println(" V ");
    SerialUSB.print("Idc : "); SerialUSB.print(pzemSlaveAddr2); SerialUSB.print(" : "); SerialUSB.print(PZEMCurrent2); SerialUSB.println(" A ");
    SerialUSB.print("Power : "); SerialUSB.print(pzemSlaveAddr2); SerialUSB.print(" : "); SerialUSB.print(PZEMPower2); SerialUSB.println(" W ");
    SerialUSB.print("Energy : "); SerialUSB.print(pzemSlaveAddr2); SerialUSB.print(" : "); SerialUSB.print(PZEMEnergy2); SerialUSB.println(" Wh ");
  }
}

void preTransmission1()                                                                                    /* transmission program when triggered*/
{
  /* 1- PZEM-017 DC Energy Meter */
  if (millis() - startMillis1 > 5000)                                                               // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    digitalWrite(MAX485_RE1, 1);                                                                     /* put RE Pin to high*/
    digitalWrite(MAX485_DE1, 1);                                                                     /* put DE Pin to high*/
    delay(1);                                                                                       // When both RE and DE Pin are high, converter is allow to transmit communication
  }
}

void postTransmission1()                                                                                   /* Reception program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */
  if (millis() - startMillis1 > 5000)                                                               // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    delay(3);                                                                                       // When both RE and DE Pin are low, converter is allow to receive communication
    digitalWrite(MAX485_RE1, 0);                                                                     /* put RE Pin to low*/
    digitalWrite(MAX485_DE1, 0);                                                                     /* put DE Pin to low*/
  }
}

void setShunt1(uint8_t slaveAddr)                                                                          //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003;                                                         /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);                                                         // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission1();                                                                                /* trigger transmission mode*/

  Serial1.write(slaveAddr);                                                                      /* these whole process code sequence refer to manual*/
  Serial1.write(SlaveParameter);
  Serial1.write(highByte(registerAddress));
  Serial1.write(lowByte(registerAddress));
  Serial1.write(highByte(NewshuntAddr));
  Serial1.write(lowByte(NewshuntAddr));
  Serial1.write(lowByte(u16CRC));
  Serial1.write(highByte(u16CRC));
  delay(10);
  postTransmission1();                                                                               /* trigger reception mode*/
  delay(100);
}

void resetEnergy1(uint8_t pzemSlaveAddr)                                               // reset energy for Meter 1
{
  uint16_t u16CRC = 0xFFFF;                         /* declare CRC check 16 bits*/
  static uint8_t resetCommand = 0x42;               /* reset command code*/
  uint8_t slaveAddr = pzemSlaveAddr;                 // if you set different address, make sure this slaveAddr must change also
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission1();                                /* trigger transmission mode*/
  Serial1.write(slaveAddr);                      /* send device address in 8 bit*/
  Serial1.write(resetCommand);                   /* send reset command */
  Serial1.write(lowByte(u16CRC));                /* send CRC check code low byte  (1st part) */
  Serial1.write(highByte(u16CRC));               /* send CRC check code high byte (2nd part) */
  delay(10);
  postTransmission1();                               /* trigger reception mode*/
  delay(100);
}

void changeAddress1(uint8_t OldslaveAddr, uint8_t NewslaveAddr)                                            //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0002;                                                         /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr);                                                      // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  preTransmission1();                                                                                 /* trigger transmission mode*/
  Serial1.write(OldslaveAddr);                                                                       /* these whole process code sequence refer to manual*/
  Serial1.write(SlaveParameter);
  Serial1.write(highByte(registerAddress));
  Serial1.write(lowByte(registerAddress));
  Serial1.write(highByte(NewslaveAddr));
  Serial1.write(lowByte(NewslaveAddr));
  Serial1.write(lowByte(u16CRC));
  Serial1.write(highByte(u16CRC));
  delay(10);
  postTransmission1();                                                                                /* trigger reception mode*/
  delay(100);
}

void preTransmission2()                                                                                    /* transmission program when triggered*/
{
  /* 1- PZEM-017 DC Energy Meter */
  if (millis() - startMillis2 > 5000)                                                               // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    digitalWrite(MAX485_RE2, 1);                                                                     /* put RE Pin to high*/
    digitalWrite(MAX485_DE2, 1);                                                                     /* put DE Pin to high*/
    delay(1);                                                                                       // When both RE and DE Pin are high, converter is allow to transmit communication
  }
}

void postTransmission2()                                                                                   /* Reception program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */
  if (millis() - startMillis2 > 5000)                                                               // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    delay(3);                                                                                       // When both RE and DE Pin are low, converter is allow to receive communication
    digitalWrite(MAX485_RE2, 0);                                                                     /* put RE Pin to low*/
    digitalWrite(MAX485_DE2, 0);                                                                     /* put DE Pin to low*/
  }
}

void setShunt2(uint8_t slaveAddr)                                                                          //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003;                                                         /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);                                                         // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission2();                                                                                /* trigger transmission mode*/

  Serial3.write(slaveAddr);                                                                      /* these whole process code sequence refer to manual*/
  Serial3.write(SlaveParameter);
  Serial3.write(highByte(registerAddress));
  Serial3.write(lowByte(registerAddress));
  Serial3.write(highByte(NewshuntAddr));
  Serial3.write(lowByte(NewshuntAddr));
  Serial3.write(lowByte(u16CRC));
  Serial3.write(highByte(u16CRC));
  delay(10);
  postTransmission2();                                                                               /* trigger reception mode*/
  delay(100);
}

void resetEnergy2(uint8_t pzemSlaveAddr)                                               // reset energy for Meter 1
{
  uint16_t u16CRC = 0xFFFF;                         /* declare CRC check 16 bits*/
  static uint8_t resetCommand = 0x42;               /* reset command code*/
  uint8_t slaveAddr = pzemSlaveAddr;                 // if you set different address, make sure this slaveAddr must change also
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission2();                                /* trigger transmission mode*/
  Serial3.write(slaveAddr);                      /* send device address in 8 bit*/
  Serial3.write(resetCommand);                   /* send reset command */
  Serial3.write(lowByte(u16CRC));                /* send CRC check code low byte  (1st part) */
  Serial3.write(highByte(u16CRC));               /* send CRC check code high byte (2nd part) */
  delay(10);
  postTransmission2();                               /* trigger reception mode*/
  delay(100);
}

void changeAddress2(uint8_t OldslaveAddr, uint8_t NewslaveAddr)                                            //Change the slave address of a mbm
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0002;                                                         /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr);                                                      // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  preTransmission2();                                                                                 /* trigger transmission mode*/
  Serial3.write(OldslaveAddr);                                                                       /* these whole process code sequence refer to manual*/
  Serial3.write(SlaveParameter);
  Serial3.write(highByte(registerAddress));
  Serial3.write(lowByte(registerAddress));
  Serial3.write(highByte(NewslaveAddr));
  Serial3.write(lowByte(NewslaveAddr));
  Serial3.write(lowByte(u16CRC));
  Serial3.write(highByte(u16CRC));
  delay(10);
  postTransmission2();                                                                                /* trigger reception mode*/
  delay(100);
}

void SERCOM2_Handler()  // Interrupt handler for SERCOM1
{
  Serial3.IrqHandler();
}