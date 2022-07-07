// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef _FOC_PROTOCOL_H
#define _FOC_PROTOCOL_H

#define HB_START_FRAME 0xABCD
#define IMU_START_FRAME 0xBCDE

typedef struct {
  uint16_t start;
  int16_t left;
  int16_t right;
  uint16_t checksum;
} SerialCommand;

typedef struct {
  uint16_t start;
  int16_t leftSpeed;
  int16_t rightSpeed;
  uint16_t leftTicks;
  uint16_t rightTicks;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t checksum;
} SerialFeedback;

typedef struct {
  float x, y, z;
} vector_t;

typedef struct {
  uint16_t start;
  vector_t mag;
  vector_t gyro;
  vector_t accel;
  uint16_t checksum;
} ImuData;

#endif
