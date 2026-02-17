#include <LTC2944.h>

const unsigned int fullCapacity = 3600;  // Maximum value is 22000 mAh

LTC2944 gauge(50);  // Takes R_SENSE value (in milliohms) as constructor argument, can be omitted if using CJMCU-294

void setup() {
  Serial.begin(115200);
  Serial.println("LTC2944 Battery Readings Example");
  Serial.println();

  Wire.begin();

  while (gauge.begin() == false) {
    Serial.println("Failed to detect LTC2944!");
    delay(5000);
  }

  gauge.setBatteryCapacity(fullCapacity);
  // gauge.setBatteryToFull(); // Sets accumulated charge registers to the maximum value
  gauge.setADCMode(ADC_MODE_AUTO);  // In sleep mode, voltage and temperature measurements will only take place when requested
  gauge.startMeasurement();
}

void loop() {
  unsigned int raw = gauge.getRawAccumulatedCharge();
  Serial.print(F("Raw Accumulated Charge: "));
  Serial.println(raw, DEC);

  float capacity = gauge.getRemainingCapacity();
  Serial.print(F("Battery Capacity: "));
  Serial.print(capacity, 3);
  Serial.print(F(" / "));
  Serial.print(fullCapacity, DEC);
  Serial.println(F(" mAh"));

  float voltage = gauge.getVoltage();
  Serial.print(F("Voltage: "));
  Serial.print(voltage - (voltage * 4.26 / 100), 3);
  // Serial.print(F(" > "));
  // Serial.print(voltage, 3);
  // Serial.print(F(" < "));
  // Serial.print(voltage + (voltage * 4.26 / 100), 3);
  Serial.println(F(" V"));

  float power = 48;
  Serial.print(F("Voltage Error: "));
  Serial.print(((voltage - power)/power) * 100, 3);
  Serial.println(F(" %"));

  float temperature = gauge.getTemperature();
  Serial.print(F("Temperature: "));
  Serial.print(temperature, 2);
  Serial.println(F(" 'C"));

  float current = gauge.getCurrent();
  Serial.print(F("Current: "));
  Serial.print(current, 3);
  Serial.println(F(" A"));

  Serial.println();

  delay(1000);
}
