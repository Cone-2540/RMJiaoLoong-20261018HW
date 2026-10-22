//
// Created by 59088 on 2026/10/18.
//

#ifndef BMI088_H
#define BMI088_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    float acc[3];  /* m/s^2, sensor axes */
    float gyro[3]; /* rad/s, sensor axes */
} Bmi088Sample;

/* Initialize before sampling. No blocking register access while DMA is active. */
uint8_t bmi088_read_sample(Bmi088Sample *sample);
uint8_t bmi088_start_dma(void);
uint8_t bmi088_take_sample(Bmi088Sample *sample);
void bmi088_dma_service(void);
extern volatile uint32_t bmi088_dma_state; /* 0 idle, 1 ACC, 2 GYRO, 3 ready, 4 error */
extern volatile uint32_t bmi088_dma_count;

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

#ifdef __cplusplus
}
#endif
#endif
