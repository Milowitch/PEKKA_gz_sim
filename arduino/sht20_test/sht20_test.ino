#include "DFRobot_SHT20.h"
// #include "wiring_private.h"  // For pinPeripheral

// #define SDA2_PIN 11
// #define SCL2_PIN 13

// TwoWire myWire(&sercom1, SDA2_PIN, SCL2_PIN); //create an I2C on SERCOM5, PA16, PA17    

DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  SerialUSB.println("SerialUSB setup done!");

  Wire.begin();

  // pinPeripheral(SDA2_PIN, PIO_SERCOM);
  // pinPeripheral(SCL2_PIN, PIO_SERCOM);
  // Init SHT20 Sensor
  sht20.initSHT20();
  delay(100);

  /**
   * Check the current status information of SHT20
   * Status information: End of battery, Heater enabled, Disable OTP reload
   * Check result: yes, no
   */
  sht20.checkSHT20();

  SerialUSB.println("Sensor init finish!");
}

void loop() {
  /**
   * Read the measured data of air humidity
   * Return the measured air humidity data of float type, unit: %
   */
  float humd = sht20.readHumidity();

  /**
   * Read the measured temp data
   * Return the measured temp data of float type, unit: C
   */
  float temp = sht20.readTemperature();

  SerialUSB.print("Time:");
  SerialUSB.print(millis());  // Get the system time from Arduino
  SerialUSB.print(" Temperature:");
  SerialUSB.print(temp, 1);  // Only print one decimal place
  SerialUSB.print("C");
  SerialUSB.print(" Humidity:");
  SerialUSB.print(humd, 1);  // Only print one decimal place
  SerialUSB.print("%");
  SerialUSB.println();

  delay(1000);
}
