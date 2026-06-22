#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * HC-SR04 status values
 */
#define HCSR04_OK               0
#define HCSR04_TIMEOUT_ERROR    1

/*
 * Returned distance when sensor reading fails
 */
#define HCSR04_ERROR_DISTANCE_CM 999

/*
 * HC-SR04 sensor structure
 */
typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;

    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;

    uint32_t distance_cm;
    uint8_t status;

} HCSR04_Sensor;

/*
 * Initialize HC-SR04 driver with timer handle.
 * TIM2 should be configured as 1 MHz timer.
 */
void HCSR04_Init(TIM_HandleTypeDef *htim);

/*
 * Read one HC-SR04 sensor distance in centimeters.
 */
uint32_t HCSR04_Read_Distance(HCSR04_Sensor *sensor);

/*
 * Read all sensors one by one.
 */
void HCSR04_Read_All(HCSR04_Sensor sensors[], uint8_t sensor_count);

/*
 * Microsecond delay using configured timer.
 */
void HCSR04_Delay_us(uint16_t us);

#ifdef __cplusplus
}
#endif

#endif /* INC_HCSR04_H_ */
