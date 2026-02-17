#include <TinyGPS++.h>

#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3

// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

#define RX1pin 4
#define TX1pin 2
// HardwareSerial neogps(1);

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  //Begin serial communication Arduino IDE (Serial Monitor)
  // Serial1.begin(9600, SERIAL_8N1, RX1pin, TX1pin);
  Serial2.begin(9600);
}

void loop() {
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    if (Serial2.available()) {
      Serial.write(Serial2.read());
      // if (gps.encode(Serial1.read())) {
        // newData = true;
      // }
    }
  }

  //If newData is true
  if (newData == true) {
    newData = false;
    Serial.print("Satellites ");
    Serial.println(gps.satellites.value());
    Serial.print("Lat ");
    Serial.println(gps.location.lat(), 5);
    Serial.print("Lng ");
    Serial.println(gps.location.lng(), 5);
    Serial.print("Speed KMPH ");
    Serial.println(gps.speed.kmph());
    //print_speed();
  } else {
  }
  delay(1);
}

void SERCOM1_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}