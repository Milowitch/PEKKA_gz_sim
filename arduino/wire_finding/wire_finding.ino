#include <Wire.h>

void setup() {
  SerialUSB.begin(115200);
  while (!Serial); // รอให้ Serial พร้อม
  SerialUSB.println("\nI2C Scanner");

  Wire.begin(); // เริ่มต้น I2C
}

void loop() {
  SerialUSB.println("Scanning...");

  int nDevices = 0; // ตัวนับจำนวนอุปกรณ์ I2C

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      SerialUSB.print("I2C device found at address 0x");
      if (address < 16)
        SerialUSB.print("0"); // เพิ่ม 0 นำหน้าถ้าค่าน้อยกว่า 16
      SerialUSB.println(address, HEX);

      nDevices++;
    } else if (error == 4) {
      SerialUSB.print("Unknown error at address 0x");
      if (address < 16)
        SerialUSB.print("0");
      SerialUSB.println(address, HEX);
    }
  }

  if (nDevices == 0)
    SerialUSB.println("No I2C devices found\n");
  else
    SerialUSB.println("Done\n");

  delay(500);
}
