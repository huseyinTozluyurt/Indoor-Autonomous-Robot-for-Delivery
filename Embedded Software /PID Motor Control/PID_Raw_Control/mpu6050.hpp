#include <Wire.h>
#include <Arduino.h>

#define MPU6050_ADDR  0x68
#define REG_WHO_AM_I  0x75
#define REG_SMPLRT_DIV       0x19  // Örnekleme hızı bölücü
#define REG_CONFIG           0x1A  // Düşük geçiren filtre (DLPF) ayarları
#define REG_GYRO_CONFIG      0x1B  // Jiroskop tam ölçek aralığı
#define REG_ACCEL_CONFIG     0x1C  // İvmeölçer tam ölçek aralığı
#define REG_ACCEL_XOUT_H     0x3B  // İvme verilerinin başladığı ilk adres
#define REG_PWR_MGMT_1       0x6B  // Güç yönetimi 


typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} MPU6050_Data_t;




void MPU6050_Init(void);
uint8_t MPU6050_Check(void);
void MPU6050_Read_All(MPU6050_Data_t *data);








//Part 2

typedef struct {
    float ax, ay, az;
    float gx , gy, gz;
    float temp;
    float roll, pitch; 
    float roll_filt, pitch_filt;
} MPU6050_Scaled_t;

void MPU6050_Get_Scaled(MPU6050_Data_t *raw, MPU6050_Scaled_t *scaled);
void MPU6050_Process(MPU6050_Data_t *raw, MPU6050_Scaled_t *clean);



//Part 3 
void MPU6050_Complementaryfilter( MPU6050_Scaled_t *clean, float dt);
