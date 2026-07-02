#include "mpu6050.hpp"

DeliveryBotMPU6050::DeliveryBotMPU6050(uint8_t i2cAddress)
  : _address(i2cAddress),
    _wire(&Wire),
    _whoAmI(0),
    _connected(false),
    _axG(0.0f), _ayG(0.0f), _azG(0.0f),
    _gxDps(0.0f), _gyDps(0.0f), _gzDps(0.0f),
    _temperatureC(0.0f),
    _gxBiasDps(0.0f), _gyBiasDps(0.0f), _gzBiasDps(0.0f),
    _rollDeg(0.0f), _pitchDeg(0.0f), _yawDeg(0.0f), _dtSec(0.0f),
    _lastUpdateMicros(0) {
}

DeliveryBotMPU6050::InitStatus DeliveryBotMPU6050::begin(TwoWire& wirePort) {
  _wire = &wirePort;
  _connected = false;

  // Wake up device. 0x00 uses internal 8 MHz oscillator.
  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
    return INIT_WIRE_ERROR;
  }
  delay(100);

  if (!readRegister(REG_WHO_AM_I, _whoAmI)) {
    return INIT_WIRE_ERROR;
  }

  if (!acceptWhoAmI(_whoAmI)) {
    return INIT_WHO_AM_I_ERROR;
  }

  // DLPF config: 0x03 is a good stable starting point for mobile robots.
  // It reduces gyro noise without making response too slow.
  writeRegister(REG_CONFIG, 0x03);

  // Sample rate divider. Gyro output rate with DLPF enabled is 1 kHz.
  // 0x04 gives about 200 Hz internal sample rate.
  writeRegister(REG_SMPLRT_DIV, 0x04);

  // Gyro full scale: ±250 deg/s, sensitivity 131 LSB/(deg/s).
  writeRegister(REG_GYRO_CONFIG, 0x00);

  // Accel full scale: ±2g, sensitivity 16384 LSB/g.
  writeRegister(REG_ACCEL_CONFIG, 0x00);

  _lastUpdateMicros = micros();
  _connected = true;
  return INIT_OK;
}

bool DeliveryBotMPU6050::calibrateGyro(uint16_t samples, uint16_t sampleDelayMs) {
  if (!_connected || samples == 0) {
    return false;
  }

  long gxSum = 0;
  long gySum = 0;
  long gzSum = 0;

  for (uint16_t i = 0; i < samples; i++) {
    int16_t axRaw, ayRaw, azRaw, tempRaw, gxRaw, gyRaw, gzRaw;
    if (!readRaw(axRaw, ayRaw, azRaw, tempRaw, gxRaw, gyRaw, gzRaw)) {
      return false;
    }

    gxSum += gxRaw;
    gySum += gyRaw;
    gzSum += gzRaw;
    delay(sampleDelayMs);
  }

  const float gyroScale = 131.0f;
  _gxBiasDps = (float)gxSum / (float)samples / gyroScale;
  _gyBiasDps = (float)gySum / (float)samples / gyroScale;
  _gzBiasDps = (float)gzSum / (float)samples / gyroScale;

  _lastUpdateMicros = micros();
  resetYaw(0.0f);
  return true;
}

bool DeliveryBotMPU6050::update() {
  if (!_connected) {
    return false;
  }

  int16_t axRaw, ayRaw, azRaw, tempRaw, gxRaw, gyRaw, gzRaw;
  if (!readRaw(axRaw, ayRaw, azRaw, tempRaw, gxRaw, gyRaw, gzRaw)) {
    return false;
  }

  unsigned long nowMicros = micros();
  unsigned long elapsedMicros = nowMicros - _lastUpdateMicros;
  _lastUpdateMicros = nowMicros;

  _dtSec = (float)elapsedMicros / 1000000.0f;

  // Ignore impossible dt values after reset or serial stalls.
  if (_dtSec <= 0.0f || _dtSec > 0.25f) {
    _dtSec = 0.0f;
  }

  const float accelScale = 16384.0f;
  const float gyroScale = 131.0f;

  _axG = (float)axRaw / accelScale;
  _ayG = (float)ayRaw / accelScale;
  _azG = (float)azRaw / accelScale;

  _gxDps = ((float)gxRaw / gyroScale) - _gxBiasDps;
  _gyDps = ((float)gyRaw / gyroScale) - _gyBiasDps;
  _gzDps = ((float)gzRaw / gyroScale) - _gzBiasDps;

  _temperatureC = ((float)tempRaw / 340.0f) + 36.53f;

  // Roll and pitch from accelerometer. These are only diagnostics for now.
  _rollDeg = atan2(_ayG, _azG) * 57.2957795f;
  _pitchDeg = atan2(-_axG, sqrt(_ayG * _ayG + _azG * _azG)) * 57.2957795f;

  // Yaw from gyro Z integration. This is the value for yaw-hold PID.
  if (_dtSec > 0.0f) {
    _yawDeg += _gzDps * _dtSec;

    // Keep yaw in a manageable range.
    if (_yawDeg > 180.0f) {
      _yawDeg -= 360.0f;
    } else if (_yawDeg < -180.0f) {
      _yawDeg += 360.0f;
    }
  }

  return true;
}

void DeliveryBotMPU6050::resetYaw(float yawDeg) {
  _yawDeg = yawDeg;
  _lastUpdateMicros = micros();
}

void DeliveryBotMPU6050::setGyroBias(float gxBiasDps, float gyBiasDps, float gzBiasDps) {
  _gxBiasDps = gxBiasDps;
  _gyBiasDps = gyBiasDps;
  _gzBiasDps = gzBiasDps;
}

float DeliveryBotMPU6050::getYawDeg() const { return _yawDeg; }
float DeliveryBotMPU6050::getRollDeg() const { return _rollDeg; }
float DeliveryBotMPU6050::getPitchDeg() const { return _pitchDeg; }
float DeliveryBotMPU6050::getGyroXDps() const { return _gxDps; }
float DeliveryBotMPU6050::getGyroYDps() const { return _gyDps; }
float DeliveryBotMPU6050::getGyroZDps() const { return _gzDps; }
float DeliveryBotMPU6050::getAccelXG() const { return _axG; }
float DeliveryBotMPU6050::getAccelYG() const { return _ayG; }
float DeliveryBotMPU6050::getAccelZG() const { return _azG; }
float DeliveryBotMPU6050::getTemperatureC() const { return _temperatureC; }
float DeliveryBotMPU6050::getDeltaTimeSec() const { return _dtSec; }
uint8_t DeliveryBotMPU6050::getWhoAmI() const { return _whoAmI; }
bool DeliveryBotMPU6050::isConnected() const { return _connected; }

bool DeliveryBotMPU6050::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  return (_wire->endTransmission() == 0);
}

bool DeliveryBotMPU6050::readRegister(uint8_t reg, uint8_t& value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  uint8_t count = _wire->requestFrom((int)_address, 1);
  if (count != 1) {
    return false;
  }

  value = _wire->read();
  return true;
}

bool DeliveryBotMPU6050::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                                 int16_t& temp,
                                 int16_t& gx, int16_t& gy, int16_t& gz) {
  _wire->beginTransmission(_address);
  _wire->write(REG_ACCEL_XOUT_H);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  uint8_t count = _wire->requestFrom((int)_address, 14);
  if (count != 14) {
    return false;
  }

  ax   = ((int16_t)_wire->read() << 8) | _wire->read();
  ay   = ((int16_t)_wire->read() << 8) | _wire->read();
  az   = ((int16_t)_wire->read() << 8) | _wire->read();
  temp = ((int16_t)_wire->read() << 8) | _wire->read();
  gx   = ((int16_t)_wire->read() << 8) | _wire->read();
  gy   = ((int16_t)_wire->read() << 8) | _wire->read();
  gz   = ((int16_t)_wire->read() << 8) | _wire->read();

  return true;
}

bool DeliveryBotMPU6050::acceptWhoAmI(uint8_t value) const {
  // Real MPU6050 usually returns 0x68.
  // Some compatible boards/chips return 0x70, which you already observed earlier.
  return (value == 0x68 || value == 0x69 || value == 0x70);
}