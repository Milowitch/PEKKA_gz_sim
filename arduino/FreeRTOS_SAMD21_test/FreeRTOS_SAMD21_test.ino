#include <FreeRTOS_SAMD21.h>

// Serial3 pin and pad definitions (in Arduino files Variant.h & modified Variant.cpp)
#define PIN_SERIAL3_RX       (45ul)               // Pin description number for PIO_SERCOM on D5
#define PIN_SERIAL3_TX       (44ul)               // Pin description number for PIO_SERCOM on D2
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3
// Instantiate the Serial3 class
Uart Serial3(&sercom2, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);

// ตัวแปรนับจำนวน
volatile int counter = 0;

void Task1(void *pvParameters) {
  while (1) {
    counter++;
    String msg = "Count: " + String(counter);

    SerialUSB.print("Task1 -> Serial1: ");
    SerialUSB.println(msg);

    Serial1.println(msg);  // ส่งไปยัง Serial3

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task2(void *pvParameters) {
  while (1) {
    if (Serial1.available()) {  // อ่านจาก Serial1
      String received = Serial1.readStringUntil('\n');
      SerialUSB.print("Task2 <- Serial1: ");
      SerialUSB.println(received);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task3(void *pvParameters) {
  while (1) {
    if (Serial3.available()) {  // อ่านจาก Serial3
      String msg = Serial3.readStringUntil('\n');
      SerialUSB.print("Task3 <- Serial3: ");
      SerialUSB.println(msg);

      SerialUSB.println("Task3 -> Serial3: ส่ง wow");
      Serial3.println("wow");  // ส่งไปยัง Serial1
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  SerialUSB.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);

  xTaskCreate(Task1, "Task1", 128, NULL, 1, NULL);
  xTaskCreate(Task2, "Task2", 128, NULL, 1, NULL);
  xTaskCreate(Task3, "Task3", 128, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS ควบคุม Task เอง
}


void SERCOM2_Handler()    // Interrupt handler for SERCOM2
{
  Serial3.IrqHandler();
}