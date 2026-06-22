#include "mpu6050.hpp"

//Part1
uint8_t MPU6050_Check() {
  uint8_t device_id = 0;
  
  Wire.beginTransmission((uint8_t)MPU6050_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false); // Bağlantıyı koparma (Restart)
  
  Wire.requestFrom((uint8_t)MPU6050_ADDR, 1U);    // 1 bayt veri iste
  if (Wire.available()) {
    device_id = Wire.read();
  }
  
  return device_id;
}

void mpu6050_write(uint8_t regaddr , uint8_t data)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(regaddr);
  Wire.write(data);
  Wire.endTransmission();
}


void mpu_read_burst(uint8_t reg, uint8_t *buffer, uint8_t size) {
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); 

  // Belirtilen boyutta veriyi talep et
  Wire.requestFrom((uint8_t)MPU6050_ADDR, size);
  
  uint8_t i = 0;
  while (Wire.available() && i < size) {
    buffer[i++] = Wire.read(); // Gelen baytları buffer'a doldur
  }
}


void MPU6050_Init(void)
{
// Uyku modunu kapat ve saati ayarla
  mpu6050_write(REG_PWR_MGMT_1, 0x01); 
  
  // Örnekleme hızını böl (Sample Rate Divider)
  mpu6050_write(REG_SMPLRT_DIV, 0x07);
  
  // İvmeölçer hassasiyeti (+-8g)
  mpu6050_write(REG_ACCEL_CONFIG, 0x10);
  
  // Jiroskop hassasiyeti (+-500 deg/s)
  mpu6050_write(REG_GYRO_CONFIG, 0x08);
}

void MPU6050_Read_All(MPU6050_Data_t* SensorData) {
    uint8_t rx_buffer[14];
    
    // 0x3B (ACCEL_XOUT_H) adresinden başlayarak 14 bayt oku
    mpu_read_burst(REG_ACCEL_XOUT_H, rx_buffer, 14);
    
    // High (Yüksek) ve Low (Düşük) baytları birleştirerek 16-bitlik ham veriyi elde et
    SensorData->Accel_X = (int16_t)(rx_buffer[0] << 8 | rx_buffer[1]);
    SensorData->Accel_Y = (int16_t)(rx_buffer[2] << 8 | rx_buffer[3]);
    SensorData->Accel_Z = (int16_t)(rx_buffer[4] << 8 | rx_buffer[5]);
    
    SensorData->Temp    = (int16_t)(rx_buffer[6] << 8 | rx_buffer[7]);
    
    SensorData->Gyro_X  = (int16_t)(rx_buffer[8] << 8 | rx_buffer[9]);
    SensorData->Gyro_Y  = (int16_t)(rx_buffer[10] << 8 | rx_buffer[11]);
    SensorData->Gyro_Z  = (int16_t)(rx_buffer[12] << 8 | rx_buffer[13]);
}


//Part2
void MPU6050_Get_Scaled(MPU6050_Data_t *raw, MPU6050_Scaled_t *scaled) {
    // +-8g ayarı için LSB Hassasiyeti: 4096 LSB/g
    scaled->ax = (float)raw->Accel_X / 4096.0f;
    scaled->ay = (float)raw->Accel_Y / 4096.0f;
    scaled->az = (float)raw->Accel_Z / 4096.0f;

    // +-500 deg/s ayarı için LSB Hassasiyeti: 65.5 LSB/deg/s
    scaled->gx = (float)raw->Gyro_X / 65.5f;
    scaled->gy = (float)raw->Gyro_Y / 65.5f;
    scaled->gz = (float)raw->Gyro_Z / 65.5f;

    // Sıcaklık formülü (Datasheet'ten)
    scaled->temp = ((float)raw->Temp / 340.0f) + 36.53f;
    scaled->roll = atan2(scaled->ay, scaled->az) * 180.0 / PI;
    scaled->pitch = atan2(-scaled->ax, sqrt(scaled->ay * scaled->ay + scaled->az * scaled->az)) * 180.0 / PI;
}

//Part3
void MPU6050_Complementaryfilter(MPU6050_Scaled_t *clean, float dt)
{
  float alpha = 0.80;
  clean->roll_filt = alpha * (clean->roll_filt + clean->gx * dt) + (1.0 - alpha) * clean->roll;
  clean->pitch_filt = alpha * (clean->pitch_filt + clean->gy * dt) + (1.0 - alpha) * clean->pitch;
}
