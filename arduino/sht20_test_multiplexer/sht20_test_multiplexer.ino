#include "DFRobot_SHT20.h"

#define A0_PIN 2
#define A1_PIN 3
#define A2_PIN 4

// DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
DFRobot_SHT20 sht20_0(&Wire, SHT20_I2C_ADDR);
DFRobot_SHT20 sht20_1(&Wire, SHT20_I2C_ADDR);

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  // Serial.println(bus);
}

void setup() {
  Serial.begin(115200);

  // pinMode(A0_PIN, OUTPUT);
  // pinMode(A1_PIN, OUTPUT);
  // pinMode(A2_PIN, OUTPUT);
  // digitalWrite(A0_PIN, 0);
  // digitalWrite(A1_PIN, 0);
  // digitalWrite(A2_PIN, 0);

  Wire.begin();

  TCA9548A(0);
  // sht20.initSHT20();
  sht20_0.initSHT20();
  delay(100);
  sht20_0.checkSHT20();

  // digitalWrite(A0_PIN, 1);
  // digitalWrite(A1_PIN, 0);
  // digitalWrite(A2_PIN, 0);
  TCA9548A(1);
  sht20_1.initSHT20();
  delay(100);
  // sht20.checkSHT20();

  sht20_1.checkSHT20();
  Serial.println("setup done!");
}

unsigned long currentmilli = millis();
unsigned long premilli[3] = { 0, 0, 0 };

void loop() {
  currentmilli = millis();
  if (currentmilli - premilli[0] >= 5000) {
    premilli[0] = currentmilli;
    
    TCA9548A(0);
    // printTempSensor(1);
    float humd = sht20_0.readHumidity();
    float temp = sht20_0.readTemperature();
    Serial.print("Sensor ");
    Serial.print("1");
    Serial.print(": Temperature: ");
    Serial.print(temp);
    Serial.print(" C, Humidity: ");
    Serial.print(humd);
    Serial.println(" %");

    findaddress();
  }
  if (currentmilli - premilli[1] >= 5100) {
    premilli[1] = currentmilli;

    TCA9548A(1);
    float humd1 = sht20_1.readHumidity();
    float temp1 = sht20_1.readTemperature();
    Serial.print("Sensor ");
    Serial.print("2");
    Serial.print(": Temperature: ");
    Serial.print(temp1);
    Serial.print(" C, Humidity: ");
    Serial.print(humd1);
    Serial.println(" %");

    findaddress();
  }
  if (currentmilli - premilli[2] >= 6000) {
    premilli[2] = currentmilli;

    // printTempSensor(2);

    // findaddress();
  }
}

// void printTempSensor(int number) {
//   float humd = sht20.readHumidity();
//   float temp = sht20.readTemperature();
//   Serial.print("Sensor ");
//   Serial.print(number);
//   Serial.print(": Temperature: ");
//   Serial.print(temp);
//   Serial.print(" C, Humidity: ");
//   Serial.print(humd);
//   Serial.println(" %");
// }

void findaddress() {
  for (int i2cAddress = 0x00; i2cAddress < 0x80; i2cAddress++) {
    Wire.beginTransmission(i2cAddress);
    if (Wire.endTransmission() == 0x00) {
      // SerialUSB.print("Wire Device found at address: 0x");
      // SerialUSB.println(i2cAddress, HEX);
      Serial.print("Wire Device found at address: 0x");
      Serial.println(i2cAddress, HEX);
    }
  }
}