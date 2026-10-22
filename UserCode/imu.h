//
// Created by 59088 on 2026/10/18.
//

#ifndef INC_20261018_IMU_H
#define INC_20261018_IMU_H
#include <stdint.h>

extern volatile uint8_t bmi088_init_error;
extern volatile uint32_t bmi088_spi_tx_status;
extern volatile uint32_t bmi088_spi_rx_status;
extern volatile uint32_t bmi088_spi_error;
extern volatile uint32_t bmi088_spi_fail_count;

void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);
void bmi088_write_reg(uint8_t reg, uint8_t data);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg,uint8_t *rx_data,uint8_t length);
void bmi088_gyro_read_reg(uint8_t reg,uint8_t *rx_data,uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data);
uint8_t bmi088_init(void);
/* Blocking reads, sensor X/Y/Z axes. On SPI failure, output is unchanged. */
void acc_calculate(float acc[3]);   /* m/s^2 */
void gyro_calculate(float gyro[3]); /* rad/s; invalid range leaves output unchanged */

#endif //INC_20261018_IMU_H
