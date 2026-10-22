#include "stm32f4xx_hal.h"
#include "main.h"
#include "spi.h"
#include "imu.h"
/* Watch: init_error 1=ACC ID, 2=GYRO ID, 3=ACC power,
 * 4=ACC config, 5=GYRO config; 0 means no failed check yet. */
volatile uint8_t bmi088_init_error = 0;
volatile uint32_t bmi088_spi_tx_status = HAL_OK;
volatile uint32_t bmi088_spi_rx_status = HAL_OK;
volatile uint32_t bmi088_spi_error = HAL_SPI_ERROR_NONE;
volatile uint32_t bmi088_spi_fail_count = 0;

void bmi088_write_byte(uint8_t tx_data)
{
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
    bmi088_spi_tx_status = status;
    if (status != HAL_OK)
    {
        bmi088_spi_error = HAL_SPI_GetError(&hspi1);
        bmi088_spi_fail_count++;
    }
}

void bmi088_read_byte(uint8_t *rx_data, uint8_t length)
{
    if (rx_data == NULL || length == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        rx_data[i] = 0xFF;
    }

    HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
    bmi088_spi_rx_status = status;
    if (status != HAL_OK)
    {
        bmi088_spi_error = HAL_SPI_GetError(&hspi1);
        bmi088_spi_fail_count++;
    }
}
void bmi088_write_reg(uint8_t reg, uint8_t data)
{
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);
}


void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,
                      CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,
                      CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,
                      CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,
                      CS1_GYRO_Pin, GPIO_PIN_SET);
}


void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_reg(reg, data);

    BMI088_ACCEL_NS_H();
}

void bmi088_accel_read_reg(uint8_t reg,
                          uint8_t *rx_data,
                          uint8_t length)
{
    if (rx_data == NULL || length == 0)
    {
        return;
    }

    uint8_t dummy = 0;

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(&dummy, 1);       // 丢弃额外的 dummy byte
    bmi088_read_byte(rx_data, length); // 获取有效数据

    BMI088_ACCEL_NS_H();
}

void bmi088_gyro_read_reg(uint8_t reg,
                         uint8_t *rx_data,
                         uint8_t length)
{
    if (rx_data == NULL || length == 0)
    {
        return;
    }

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(rx_data, length);

    BMI088_GYRO_NS_H();
}

void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data)
{
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_reg(reg, tx_data);

    BMI088_GYRO_NS_H();
}

uint8_t bmi088_init(void) {
    bmi088_init_error = 0;
    uint8_t dummy = 0;
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t acc_power[2] = {0};
    uint8_t acc_config[2] = {0};
    uint8_t gyro_config[3] = {0};

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    HAL_Delay(1);

    // Switch ACCEL to SPI，第一次读到的值不用
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Soft Reset ACCEL
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(1);

    // Soft Reset GYRO
    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(30);

    // ACCEL 软复位后再做一次虚拟读取
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Check Chip ID
    bmi088_accel_read_reg(0x00, &acc_id, 1);
    if (acc_id != 0x1E) {
        bmi088_init_error = 1;
        return 0;
    }
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) {
        bmi088_init_error = 2;
        return 0;
    }

    // Switch ACCEL to Active Mode
    bmi088_accel_write_single_reg(0x7C, 0x00); // ACC_PWR_CONF
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7D, 0x04); // ACC_PWR_CTRL
    HAL_Delay(5);

    // ACCEL Range / ODR / BW
    bmi088_accel_write_single_reg(0x41, 0x01); // ACC_RANGE
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x40, 0xAB); // ACC_CONF
    HAL_Delay(40);

    // GYRO Power / Range / ODR / BW
    bmi088_gyro_write_single_reg(0x11, 0x00); // GYRO_LPM1
    HAL_Delay(30);
    bmi088_gyro_write_single_reg(0x0F, 0x00); // GYRO_RANGE
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x10, 0x02); // GYRO_BANDWIDTH
    HAL_Delay(10);

    // Read back，连续读取相邻的配置寄存器
    bmi088_accel_read_reg(0x7C, acc_power, 2);
    if (acc_power[0] != 0x00 || acc_power[1] != 0x04) {
        bmi088_init_error = 3;
        return 0;
    }
    bmi088_accel_read_reg(0x40, acc_config, 2);
    if (acc_config[0] != 0xAB || (acc_config[1] & 0x03) != 0x01) {
        bmi088_init_error = 4;
        return 0;
    }
    bmi088_gyro_read_reg(0x0F, gyro_config, 3);
    if ((gyro_config[0] & 0x07) != 0x00 ||
        (gyro_config[1] & 0x07) != 0x02 || gyro_config[2] != 0x00) {
        bmi088_init_error = 5;
        return 0;
        }

    return 1;
}

void acc_calculate(float acc[3])
{
    static const float range_g[4] = {3.0f, 6.0f, 12.0f, 24.0f};
    uint8_t raw_range = 0;
    uint8_t data[6] = {0};
    uint32_t failures = bmi088_spi_fail_count;

    if (acc == NULL)
        return;

    // ACC_RANGE bits [1:0] select the full scale; bits [7:2] are reserved.
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    if (bmi088_spi_fail_count != failures)
        return;
    raw_range &= 0x03;

    // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB.
    // bmi088_accel_read_reg() already discards the SPI dummy byte.
    bmi088_accel_read_reg(0x12, data, 6);
    if (bmi088_spi_fail_count != failures)
        return;

    float scale = range_g[raw_range] * 9.80665f / 32768.0f;
    for (uint8_t i = 0; i < 3; i++)
    {
        int16_t raw = (int16_t)(((uint16_t)data[2 * i + 1] << 8) |
                                data[2 * i]);
        acc[i] = raw * scale; // m/s^2, sensor X/Y/Z axes.
    }
}

void gyro_calculate(float gyro[3])
{
    static const float range_dps[5] = {
        2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f
    };
    uint8_t raw_range = 0;
    uint8_t data[6] = {0};
    uint32_t failures = bmi088_spi_fail_count;

    if (gyro == NULL)
        return;

    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    if (bmi088_spi_fail_count != failures || raw_range > 4)
        return;

    bmi088_gyro_read_reg(0x02, data, 6);
    if (bmi088_spi_fail_count != failures)
        return;

    float scale = range_dps[raw_range] / 32768.0f *
                  (3.14159265358979323846f / 180.0f);
    for (uint8_t i = 0; i < 3; i++)
    {
        int16_t raw = (int16_t)(((uint16_t)data[2 * i + 1] << 8) |
                                data[2 * i]);
        gyro[i] = raw * scale; // rad/s, sensor X/Y/Z axes.
    }
}

