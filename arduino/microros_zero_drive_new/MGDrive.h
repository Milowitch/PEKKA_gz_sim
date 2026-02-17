#ifndef MGDRIVE_H
#define MGDRIVE_H

#include <mcp_can.h>
#include <map>
#include <cmath>

// Define your MGDrive class
class MGDrive {
public:
  MGDrive(MCP_CAN* canInterface, int interruptPin);
  bool processMessages();
  bool Motor_off(uint8_t id);
  bool Motor_on(uint8_t id);
  bool Motor_stop(uint8_t id);
  bool Clearmotorerrorstate(uint8_t id);
  bool ReadMotorState2(uint8_t id);
  bool Speedclosedloopcontrol(uint8_t id, float speedRPM);
  bool Multiloopanglecontrol2(uint8_t id, float maxSpeedRPM, float angle);
  bool Torqueclosedloopcontrol(uint8_t id, float iq);
  bool Multimotortorqueclosedloopcontrol(float iq_1, float iq_2, float iq_3, float iq_4);

  float getTemperatureValue(uint8_t id) const;
  float getTorqueCurrentValue(uint8_t id) const;
  float getSpeedValue(uint8_t id) const;
  float getEncoderPositionValue(uint8_t id) const;

  void printMapContents() const;

private:
  MCP_CAN* _canInterface;
  const int _interruptPin;
  struct CANData {
    float temperatureValue;
    float torqueCurrentValue;
    float SpeedValue;
    float encoderPositionValue;
  };

  std::map<uint8_t, CANData> _canDataMap;
};

#endif  // MGDRIVE_H
