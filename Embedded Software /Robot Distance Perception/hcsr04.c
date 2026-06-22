#include "hcsr04.h"

/*
 * Private timer pointer used by HC-SR04 driver.
 */
static TIM_HandleTypeDef *hcsr04_timer = NULL;

/*
 * Initialize HC-SR04 driver.
 */
void HCSR04_Init(TIM_HandleTypeDef *htim)
{
    hcsr04_timer = htim;
    HAL_TIM_Base_Start(hcsr04_timer);
}

/*
 * Microsecond delay.
 * TIM2 must be configured to count at 1 MHz.
 *
 * If SYSCLK = 8 MHz:
 * TIM2 Prescaler = 7
 *
 * If SYSCLK = 72 MHz:
 * TIM2 Prescaler = 71
 */
void HCSR04_Delay_us(uint16_t us)
{
    if (hcsr04_timer == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(hcsr04_timer, 0);

    while (__HAL_TIM_GET_COUNTER(hcsr04_timer) < us)
    {
    }
}

/*
 * Read distance from one HC-SR04 sensor.
 */
uint32_t HCSR04_Read_Distance(HCSR04_Sensor *sensor)
{
    uint32_t pulse_time_us = 0;
    uint32_t timeout_start = 0;

    if (sensor == NULL || hcsr04_timer == NULL)
    {
        return HCSR04_ERROR_DISTANCE_CM;
    }

    /*
     * Make sure TRIG is LOW before starting.
     */
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);
    HCSR04_Delay_us(2);

    /*
     * Send 10 us trigger pulse.
     */
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_SET);
    HCSR04_Delay_us(10);
    HAL_GPIO_WritePin(sensor->trig_port, sensor->trig_pin, GPIO_PIN_RESET);

    /*
     * Wait for ECHO to become HIGH.
     */
    timeout_start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - timeout_start) > 50)
        {
            sensor->distance_cm = HCSR04_ERROR_DISTANCE_CM;
            sensor->status = HCSR04_TIMEOUT_ERROR;
            return sensor->distance_cm;
        }
    }

    /*
     * Measure ECHO HIGH time.
     */
    __HAL_TIM_SET_COUNTER(hcsr04_timer, 0);

    timeout_start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - timeout_start) > 50)
        {
            sensor->distance_cm = HCSR04_ERROR_DISTANCE_CM;
            sensor->status = HCSR04_TIMEOUT_ERROR;
            return sensor->distance_cm;
        }
    }

    pulse_time_us = __HAL_TIM_GET_COUNTER(hcsr04_timer);

    /*
     * HC-SR04 distance formula:
     *
     * distance_cm = echo_time_us / 58
     */
    sensor->distance_cm = pulse_time_us / 58;
    sensor->status = HCSR04_OK;

    return sensor->distance_cm;
}

/*
 * Read multiple sensors sequentially.
 *
 * Important:
 * HC-SR04 sensors should not be triggered at the same time.
 * Reading them one by one reduces ultrasonic crosstalk.
 */
void HCSR04_Read_All(HCSR04_Sensor sensors[], uint8_t sensor_count)
{
    for (uint8_t i = 0; i < sensor_count; i++)
    {
        HCSR04_Read_Distance(&sensors[i]);

        /*
         * Small delay between sensors to reduce interference.
         */
        HAL_Delay(60);
    }
}
