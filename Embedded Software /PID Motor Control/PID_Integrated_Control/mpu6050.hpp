#ifndef DELIVERYBOT_MPU6050_HPP
#define DELIVERYBOT_MPU6050_HPP

#include <Arduino.h>
#include <Wire.h>

class DeliveryBotMPU6050 {
public:
  enum InitStatus {
    INIT_OK = 0,
    INIT_WIRE_ERROR,
    INIT_WHO_AM_I_ERROR
  };

  explicit DeliveryBotMPU6050(uint8_t i2cAddress = 0x68);

  // Call once in setup().
  // On Arduino Uno, call Wire.begin() before begin().
  InitStatus begin(TwoWire& wirePort = Wire);

  // Keep the robot completely still while this runs.
  // This estimates gyro offsets, especially gyro Z bias for yaw integration.
  bool calibrateGyro(uint16_t samples = 1000, uint16_t sampleDelayMs = 2);

  // Call this as often as possible inside loop().
  // Returns true when a new IMU sample was read and integrated.
  bool update();

  // Use this when a new movement command starts.
  void resetYaw(float yawDeg = 0.0f);

  // Useful if you already measured a bias manually.
  void setGyroBias(float gxBiasDps, float gyBiasDps, float gzBiasDps);

  // Main value needed for straight-line PID.
  float getYawDeg() const;

  // Optional diagnostics.
  float getRollDeg() const;
  float getPitchDeg() const;
  float getGyroXDps() const;
  float getGyroYDps() const;
  float getGyroZDps() const;
  float getAccelXG() const;
  float getAccelYG() const;
  float getAccelZG() const;
  float getTemperatureC() const;
  float getDeltaTimeSec() const;
  uint8_t getWhoAmI() const;
  bool isConnected() const;

private:
  static const uint8_t REG_SMPLRT_DIV   = 0x19;
  static const uint8_t REG_CONFIG       = 0x1A;
  static const uint8_t REG_GYRO_CONFIG  = 0x1B;
  static const uint8_t REG_ACCEL_CONFIG = 0x1C;
  static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
  static const uint8_t REG_PWR_MGMT_1   = 0x6B;
  static const uint8_t REG_WHO_AM_I     = 0x75;

  uint8_t _address;
  TwoWire* _wire;
  uint8_t _whoAmI;
  bool _connected;

  float _axG;
  float _ayG;
  float _azG;
  float _gxDps;
  float _gyDps;
  float _gzDps;
  float _temperatureC;

  float _gxBiasDps;
  float _gyBiasDps;
  float _gzBiasDps;

  float _rollDeg;
  float _pitchDeg;
  float _yawDeg;
  float _dtSec;

  unsigned long _lastUpdateMicros;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t& value);
  bool readRaw(int16_t& ax, int16_t& ay, int16_t& az,
               int16_t& temp,
               int16_t& gx, int16_t& gy, int16_t& gz);
  bool acceptWhoAmI(uint8_t value) const;
};

#endif