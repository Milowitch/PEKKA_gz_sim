#include "DFRobot_SHT20.h"

#define INT1_PIN 2
#define INT2_PIN 3

DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

void setup() {
  Serial.begin(115200);

  sht20.initSHT20();

  pinMode(INT1_PIN, OUTPUT);
  pinMode(INT2_PIN, OUTPUT);
  digitalWrite(INT1_PIN, 0);
  digitalWrite(INT2_PIN, 0);
  // Wire.begin();

  delay(100);
  sht20.checkSHT20();
  Serial.println("setup done!");
}

unsigned long currentmilli = millis();
unsigned long premilli[2] = { 0, 0 };

void loop() {
  currentmilli = millis();
  if (currentmilli - premilli[0] >= 5000) {
    premilli[0] = currentmilli;

    digitalWrite(INT1_PIN, 0);
    digitalWrite(INT2_PIN, 0);

    printTempSensor(1);
  }
  if (currentmilli - premilli[1] >= 5100) {
    premilli[1] = currentmilli;

    digitalWrite(INT1_PIN, 1);
    digitalWrite(INT2_PIN, 1);

    printTempSensor(2);
  }
}

void printTempSensor(int number) {
  float humd = sht20.readHumidity();
  float temp = sht20.readTemperature();
  Serial.print("Sensor ");
  Serial.print(number);
  Serial.print(": Temperature: ");
  Serial.print(temp);
  Serial.print(" C, Humidity: ");
  Serial.print(humd);
  Serial.println(" %");
}
