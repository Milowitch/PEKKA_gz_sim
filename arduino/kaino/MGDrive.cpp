#include "MGDrive.h"

MGDrive::MGDrive(MCP_CAN* canInterface, const int interruptPin)
  : _canInterface(canInterface), _interruptPin(interruptPin) {
  pinMode(_interruptPin, INPUT);
}

bool MGDrive::processMessages() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  if (!digitalRead(_interruptPin)) {
    _canInterface->readMsgBuf(&rxId, &len, rxBuf);

    // Adjust CAN ID by subtracting 0x140
    rxId -= 0x140;

    // Extracting the data from the buffer
    if (rxBuf[0] == 0xA0 || rxBuf[0] == 0xA1 || rxBuf[0] == 0xA2 || rxBuf[0] == 0xA4 || rxBuf[0] == 0xA5 || rxBuf[0] == 0xA6 || rxBuf[0] == 0xA7 || rxBuf[0] == 0xA8 || rxBuf[0] == 0x9C) {
      int8_t temperature = rxBuf[1];
      int16_t torqueCurrent = (rxBuf[3] << 8) | rxBuf[2];     // High byte and low byte combined
      int16_t motorSpeed = (rxBuf[5] << 8) | rxBuf[4];        // High byte and low byte combined
      uint16_t encoderPosition = (rxBuf[7] << 8) | rxBuf[6];  // High byte and low byte combined

      CANData data;
      data.temperatureValue = static_cast<float>(temperature);
      data.torqueCurrentValue = static_cast<float>(-32.0f + ((torqueCurrent + 2000.0f) / 62.5f));
      data.SpeedValue = static_cast<float>(motorSpeed / 6.0f / 6.0f);
      data.encoderPositionValue = static_cast<float>((encoderPosition / 16383.0f * 2.0f * M_PI) / 24.0f * 360.0f);

      _canDataMap[rxId] = data;

      // Serial.print("MG ID: 0x0");
      // Serial.print(rxId);
      // Serial.print(" Temperature: ");
      // Serial.print(data.temperatureValue);
      // Serial.print(" TorqueCurrent: ");
      // Serial.print(data.torqueCurrentValue);
      // Serial.print(" MotorSpeed: ");
      // Serial.print(data.SpeedValue);
      // Serial.print(" EncoderPosition: ");
      // Serial.println(data.encoderPositionValue);
    }
    return true;
  }

  return false;
}

bool MGDrive::Motor_off(uint8_t id) {
  uint8_t data[8];
  data[0] = 0x80;
  memset(&data[1], 0, 7);  // Clear the rest of the data

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Motor_on(uint8_t id) {
  uint8_t data[8];
  data[0] = 0x88;
  memset(&data[1], 0, 7);  // Clear the rest of the data

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Motor_stop(uint8_t id) {
  uint8_t data[8];
  data[0] = 0x81;
  memset(&data[1], 0, 7);  // Clear the rest of the data

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Clearmotorerrorstate(uint8_t id) {
  uint8_t data[8];
  data[0] = 0x9B;
  memset(&data[1], 0, 7);  // Clear the rest of the data

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::ReadMotorState2(uint8_t id) {
  uint8_t data[8];
  data[0] = 0x9C;
  memset(&data[1], 0, 7);  // Clear the rest of the data

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Speedclosedloopcontrol(uint8_t id, float speedRPM) {
  int32_t speedControl = static_cast<int32_t>(speedRPM * 600.0f * 6.0f);
  uint8_t data[8];
  data[0] = 0xA2;
  memcpy(&data[4], &speedControl, sizeof(int32_t));

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Multiloopanglecontrol2(uint8_t id, float maxSpeedRPM, float angle) {
  uint16_t maxSpeed = static_cast<uint16_t>(maxSpeedRPM * 6.0f * 6.0f);
  int32_t angleControl = static_cast<int32_t>(angle * 18.0f / M_PI * 100.0f);
  uint8_t data[8];
  data[0] = 0xA4;
  memcpy(&data[2], &maxSpeed, sizeof(uint16_t));
  memcpy(&data[4], &angleControl, sizeof(int32_t));
  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Torqueclosedloopcontrol(uint8_t id, float iq) {
  int16_t iqControl = static_cast<int16_t>(std::round(-2000.0f + ((iq + 32.0f) * 62.5f)));

  uint8_t data[8];
  data[0] = 0xA1;
  memcpy(&data[4], &iqControl, sizeof(iqControl));

  byte sndStat = _canInterface->sendMsgBuf(0x140 + id, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

bool MGDrive::Multimotortorqueclosedloopcontrol(float iq_1, float iq_2, float iq_3, float iq_4) {
  int16_t iqControl_1 = static_cast<int16_t>(std::round(-2000.0f + ((iq_1 + 32.0f) * 62.5f)));
  int16_t iqControl_2 = static_cast<int16_t>(std::round(-2000.0f + ((iq_2 + 32.0f) * 62.5f)));
  int16_t iqControl_3 = static_cast<int16_t>(std::round(-2000.0f + ((iq_3 + 32.0f) * 62.5f)));
  int16_t iqControl_4 = static_cast<int16_t>(std::round(-2000.0f + ((iq_4 + 32.0f) * 62.5f)));

  uint8_t data[8];
  memcpy(&data[0], &iqControl_1, sizeof(int16_t));
  memcpy(&data[2], &iqControl_2, sizeof(int16_t));
  memcpy(&data[4], &iqControl_3, sizeof(int16_t));
  memcpy(&data[6], &iqControl_4, sizeof(int16_t));

  byte sndStat = _canInterface->sendMsgBuf(0x280, 0, 8, data);
  if (sndStat == CAN_OK) {
    // Serial.println("Message Sent Successfully!");
    processMessages();
    return true;
  } else {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

float MGDrive::getTemperatureValue(uint8_t id) const {
  auto it = _canDataMap.find(id);
  if (it != _canDataMap.end()) {
    return it->second.temperatureValue;
  }
  return NAN;
}

float MGDrive::getTorqueCurrentValue(uint8_t id) const {
  auto it = _canDataMap.find(id);
  if (it != _canDataMap.end()) {
    return it->second.torqueCurrentValue;
  }
  return NAN;
}

float MGDrive::getSpeedValue(uint8_t id) const {
  auto it = _canDataMap.find(id);
  if (it != _canDataMap.end()) {
    return it->second.SpeedValue;
  }
  return NAN;
}

float MGDrive::getEncoderPositionValue(uint8_t id) const {
  auto it = _canDataMap.find(id);
  if (it != _canDataMap.end()) {
    return it->second.encoderPositionValue;
  }
  return NAN;
}