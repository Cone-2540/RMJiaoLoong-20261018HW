#include "imu_app.h"
#include "imu.hpp"
#include "bmi088.h"
#include "tim.h"

// C linkage gives Scope/Keil simple, stable symbol names. Angles are degrees.
extern "C" {
volatile float roll_deg, pitch_deg, yaw_deg;
volatile float comp_roll_deg, comp_pitch_deg, comp_yaw_deg;
volatile float mahony_roll_deg, mahony_pitch_deg, mahony_yaw_deg;
volatile float acc_x, acc_y, acc_z;             // body m/s^2
volatile float gyro_x_dps, gyro_y_dps, gyro_z_dps; // body, bias removed
volatile float bias_x_dps, bias_y_dps, bias_z_dps;
volatile float gyro_uncal_x_dps, gyro_uncal_y_dps, gyro_uncal_z_dps;
volatile int16_t acc_raw[3], gyro_raw[3];       // original sensor axes
volatile float imu_dt_ms, imu_sample_hz;
volatile uint32_t imu_init_ok, imu_acc_id, imu_gyro_id;
volatile uint32_t imu_calibrated, imu_calibration_count, imu_sample_count;
volatile uint32_t imu_skipped_ticks;
volatile uint32_t imu_algorithm = 0;           // writable: 0 complementary, 1 Mahony
volatile uint32_t imu_recalibrate = 0;         // write 1 while board is stationary
volatile uint32_t imu_use_dma = 1;             // writable: 0 blocking, 1 DMA
}

namespace {
Imu imu;
volatile uint32_t ticks;
uint32_t handled_tick, sample_cycles, last_cycles, rate_ms, rate_count;
constexpr float rad_to_deg = 57.2957795131f;

void process(const Bmi088Sample &sample)
{
    float dt = last_cycles ? (uint32_t)(sample_cycles-last_cycles)/(float)SystemCoreClock : 0.002f;
    last_cycles = sample_cycles;
    // A debugger pause must not become one enormous integration step.
    if (dt > 0.05f) { imu.reset(); dt=0.002f; }
    // This C-board lies component-side up with sensor +Z upward.
    // Body axes follow the BMI088 axes; apply any future mounting rotation here.
    imu.update(sample.acc, sample.gyro, dt);
    for (int i=0; i<3; ++i) {
        acc_raw[i]=sample.acc_raw[i]; gyro_raw[i]=sample.gyro_raw[i];
    }
    const float *a=imu.acceleration(), *w=imu.angularRate(), *b=imu.bias();
    acc_x=a[0]; acc_y=a[1]; acc_z=a[2];
    gyro_x_dps=w[0]*rad_to_deg; gyro_y_dps=w[1]*rad_to_deg; gyro_z_dps=w[2]*rad_to_deg;
    bias_x_dps=b[0]*rad_to_deg; bias_y_dps=b[1]*rad_to_deg; bias_z_dps=b[2]*rad_to_deg;
    gyro_uncal_x_dps=gyro_x_dps+bias_x_dps;
    gyro_uncal_y_dps=gyro_y_dps+bias_y_dps;
    gyro_uncal_z_dps=gyro_z_dps+bias_z_dps;
    Imu::Angles c=imu.complementary(), m=imu.mahony();
    comp_roll_deg=c.roll*rad_to_deg; comp_pitch_deg=c.pitch*rad_to_deg; comp_yaw_deg=c.yaw*rad_to_deg;
    mahony_roll_deg=m.roll*rad_to_deg; mahony_pitch_deg=m.pitch*rad_to_deg; mahony_yaw_deg=m.yaw*rad_to_deg;
    Imu::Angles e=imu_algorithm ? m : c;
    roll_deg=e.roll*rad_to_deg; pitch_deg=e.pitch*rad_to_deg; yaw_deg=e.yaw*rad_to_deg;
    imu_dt_ms=dt*1000;
    imu_calibrated=imu.calibrated(); imu_calibration_count=imu.calibrationCount();
    ++imu_sample_count;
    uint32_t now=HAL_GetTick();
    if (now-rate_ms >= 1000) {
        imu_sample_hz=(imu_sample_count-rate_count)*1000.0f/(now-rate_ms);
        rate_count=imu_sample_count; rate_ms=now;
    }
}
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM6) ++ticks; // No SPI or trigonometry in timer ISR.
}

void imu_app_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_Delay(50);
    imu_init_ok=bmi088_init();
    uint8_t a=0, g=0;
    bmi088_accel_read_reg(0x00,&a,1); bmi088_gyro_read_reg(0x00,&g,1);
    imu_acc_id=a; imu_gyro_id=g;
    rate_ms=HAL_GetTick();
    if (imu_init_ok && HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

void imu_app_poll(void)
{
    if (!imu_init_ok) return;
    bmi088_dma_service();
    Bmi088Sample sample;
    if (bmi088_take_sample(&sample)) process(sample);
    if (imu_recalibrate) {
        imu.reset(); imu_calibrated=0; imu_calibration_count=0;
        last_cycles=0; imu_recalibrate=0;
    }
    uint32_t now=ticks;
    if (now==handled_tick) return;
    imu_skipped_ticks += now-handled_tick-1;
    handled_tick=now;
    if (bmi088_dma_state != 0) { ++imu_skipped_ticks; return; }
    sample_cycles=DWT->CYCCNT;
    if (imu_use_dma) bmi088_start_dma();
    else if (bmi088_read_sample(&sample)) process(sample);
}
