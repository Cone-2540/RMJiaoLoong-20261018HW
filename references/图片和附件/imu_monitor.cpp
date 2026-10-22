/**
 ******************************************************************************
 * @file    imu_monitor.cpp/h
 * @brief   IMU管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/imu_monitor.h"
#include "app/_robot_switch.h"
#include "base/common/math.h"
#include "base/imu/driver/bmi088.h"
#include "base/imu/driver/ist8310.h"
#include "base/imu/driver/mpu6500.h"

struct ImuConfig {
  MahonyParam mahony;
  const float* r_vec;
  float accel_filter_k;
  float w_dot_filter_k;
  const float (*R_imu)[3];
  const float (*R_mag)[3];
  const float* gyro_bias;
  const float* accel_bias;
  const float* mag_bias;
  float accel_sen;
};

IMU makeImu(const ImuConfig& config,
            void (*readSensor)(imu::RawData_t&) = nullptr) {
  return IMU(config.mahony, config.r_vec, config.accel_filter_k,
             config.w_dot_filter_k, config.R_imu, config.R_mag,
             config.gyro_bias, config.accel_bias,
             config.mag_bias, config.accel_sen, readSensor);
}

// 板载IMU
namespace boardimu {

// 板载IMU传感器初始化函数
void initSensor(imu::RawData_t& raw_data);
// 板载imu传感器数据读取函数
void readSensor(imu::RawData_t& raw_data);

// 传感器开关
constexpr bool bmi088_enabled = true;
constexpr bool ist8310_enabled = false;
constexpr bool mpu6500_enabled = true;
constexpr uint8_t bmi088_init_max_retries = 3;
constexpr uint16_t bmi088_init_retry_delay_ms = 10;
uint8_t bmi088_init_error = BMI088_NO_ERROR;
bool bmi088_ready = false;

// imu参数
struct Param {
  // 姿态解算算法参数
  const float dt = 1e-3f;
  const float kg = 5e-3f;
  const float km = 0;
  const float g_norm_thres = 2.f;
  const float accel_filter_k = 1.f;
  const float w_dot_filter_k = 0.05f;

  // 传感器方向变换矩阵(对应不同安装方向，默认为I(3))
  const float R_imu[3][3] = {
    { 0.0f,  1.0f, 0.0f},  // body x
    {-1.0f,  0.0f, 0.0f},  // body y
    { 0.0f,  0.0f, 1.0f},  // body z
  };
  const float R_mag[3][3] = {{0.93f, 0.0f, 0.0f},
                             {0.0f, 1.03f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};

  // 传感器偏移
#if ROBOT_NATIONAL_SILVER
  const float gyro_bias[3] = {-0.00139999983f, -0.00465000048f,
                              -0.000999999116f};
#elif ROBOT_NATIONAL_BLACK
  const float gyro_bias[3] = {2.4e-3f, -1.85e-3f, 1.5e-3f};
#elif ROBOT_HERO_BADBOY
  const float gyro_bias[3] = {-0.0045f, -0.0002f, 0.005f};
#elif ROBOT_HERO_SIRIUS
  const float gyro_bias[3] = {-0.0045f, -0.0002f, 0.005f};
#endif
  const float accel_bias[3] = {0, 0, 0};
  const float mag_bias[3] = {-13.0f, -0.5f, 30.0f};
  // 加速度计比例
  const float accel_sen = 1;
  // 杆臂向量，指向从传感器到旋转中心
  const float r_vec[3] = {0.232f, 0.0f, 0.076f};
} param;

// IMU温度控制
imu::TempControl temp(BOARD_IMU_HEAT_TIM, BOARD_IMU_HEAT_CHANNEL,
                      PID(5000, 0, 0, 500, 5000));

}  // namespace boardimu

// 板载IMU对象
IMU board_imu = makeImu({{boardimu::param.dt,
                          boardimu::param.kg,
                          boardimu::param.km,
                          boardimu::param.g_norm_thres},
                         boardimu::param.r_vec,
                         boardimu::param.accel_filter_k,
                         boardimu::param.w_dot_filter_k,
                         boardimu::param.R_imu,
                         boardimu::param.R_mag,
                         boardimu::param.gyro_bias,
                         boardimu::param.accel_bias,
                         boardimu::param.mag_bias,
                         boardimu::param.accel_sen},
                        &boardimu::readSensor);

// 云台IMU(外置，can通信传输yaw，pitch数据)
namespace gimbalimu {

// imu参数
struct Param {
  const float dt = 1e-3f;
  const float kg = 5e-3f;
  const float km = 0;
  const float g_norm_thres = 0.4f;
  const float accel_filter_k = 1.f;
  const float w_dot_filter_k = 0.05f;

  // 传感器方向变换矩阵(对应不同安装方向，默认为I(3))
#if ROBOT_HERO_BADBOY
  const float R_imu[3][3] = {
      {0.990268f, 0.0f, -0.139173f},
      {0.0f, -1.0f, 0.0f},
      {-0.139173f, 0.0f, -0.990268f}
  };
#elif ROBOT_HERO_SIRIUS
  const float R_imu[3][3] = {
    {0.990268f, 0.0f, -0.139173f},
    {0.0f, -1.0f, 0.0f},
    {-0.139173f, 0.0f, -0.990268f}
  };
#else
  const float R_imu[3][3] = {{-1.0f, 0.0f, 0.0f},
                             {0.0f, -1.0f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};
#endif
  const float R_mag[3][3] = {{1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};

  // 传感器偏移
#if ROBOT_NATIONAL_SILVER
  const float gyro_bias[3] = {-1.95e-3f, -3.2e-3f, 1.7e-3f};
#elif ROBOT_HERO_BADBOY
  const float gyro_bias[3] = {0.0018, 0.00203, -0.00005};
#elif ROBOT_HERO_SIRIUS
  const float gyro_bias[3] = {0.0017, 0.0025, -0.0038};
#endif
  const float accel_bias[3] = {0, 0, 0};
  const float mag_bias[3] = {0, 0, 0};
  // 加速度计比例
  const float accel_sen = 1;
  const float r_vec[3] = {0, 0, 0};
} param;
}  // namespace gimbalimu

// 云台imu对象(通过can通信获取yaw，pitch数据)
IMU gimbal_imu = makeImu({{gimbalimu::param.dt,
                           gimbalimu::param.kg,
                           gimbalimu::param.km,
                           gimbalimu::param.g_norm_thres},
                          gimbalimu::param.r_vec,
                          gimbalimu::param.accel_filter_k,
                          gimbalimu::param.w_dot_filter_k,
                          gimbalimu::param.R_imu,
                          gimbalimu::param.R_mag,
                          gimbalimu::param.gyro_bias,
                          gimbalimu::param.accel_bias,
                          gimbalimu::param.mag_bias,
                          gimbalimu::param.accel_sen});

// 外置IMU
namespace extimu {
struct Param {
  // 姿态解算算法参数
  const float dt = 1e-3f;
  const float kg = 5e-3f;
  // const float kg = 0.f;

  const float km = 0;
  const float g_norm_thres = 0.4f;
  const float accel_filter_k = 1.f;
  const float w_dot_filter_k = 0.05f;

  // 传感器方向变换矩阵(对应不同安装方向，默认为I(3))
  const float R_imu[3][3] = {{-1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, -1.0f}};
  const float R_mag[3][3] = {{0.0f, 0.0f, 0.0f},
                             {0.0f, 0.0f, 0.0f},
                             {0.0f, 0.0f, 0.0f}};

  const float gyro_bias[3] = {-2.4e-3f, -5.1e-3f, -1.1e-3f};

  const float accel_bias[3] = {0, 0, 0};
  const float mag_bias[3] = {-0.0f, -0.0f, 0.0f};
  // 加速度计比例
  const float accel_sen = 1;
  const float r_vec[3] = {0.0208f, 0, 0};
} param;
}  // namespace extimu

// 外置imu对象(通过can通信获取yaw，pitch数据)
IMU ext_imu = makeImu({{extimu::param.dt,
                        extimu::param.kg,
                        extimu::param.km,
                        extimu::param.g_norm_thres},
                       extimu::param.r_vec,
                       extimu::param.accel_filter_k,
                       extimu::param.w_dot_filter_k,
                       extimu::param.R_imu,
                       extimu::param.R_mag,
                       extimu::param.gyro_bias,
                       extimu::param.accel_bias,
                       extimu::param.mag_bias,
                       extimu::param.accel_sen});

// 初始化板载IMU传感器
void boardimu::initSensor(imu::RawData_t& raw_data) {
#ifdef DBC
  if (boardimu::bmi088_enabled) {
    imu::delay(100);
    boardimu::bmi088_ready = false;
    for (uint8_t retry = 0; retry < boardimu::bmi088_init_max_retries;
         ++retry) {
      boardimu::bmi088_init_error = BMI088_init();
      if (boardimu::bmi088_init_error == BMI088_NO_ERROR) {
        boardimu::bmi088_ready = true;
        break;
      }
      imu::delay(boardimu::bmi088_init_retry_delay_ms);
    }
    if (boardimu::bmi088_ready) {
      BMI088_read(raw_data.gyro, raw_data.accel, raw_data.temp);
    }
  }
  if (boardimu::ist8310_enabled) {
    while (ist8310_init()) {
      imu::delay(10);
    }
    ist8310_read_mag(raw_data.mag);
  }
  // 使用预校准参数，不进行开机校准
  board_imu.init(EulerAngle_t(0, 0, 0), true);
#elif defined DBA
  if (boardimu::mpu6500_enabled) {
    mpu_device_init();
  }
  // 开机校准500ms
  board_imu.init(EulerAngle_t(0, 0, 0), false, 500);
#endif  // DBC / DBA
  boardimu::temp.init();
}

// 读取板载IMU传感器数据
void boardimu::readSensor(imu::RawData_t& raw_data) {
#ifdef DBC
  if (boardimu::bmi088_enabled && boardimu::bmi088_ready) {
    BMI088_read(raw_data.gyro, raw_data.accel, raw_data.temp);
  }
  if (boardimu::ist8310_enabled) {
    ist8310_read_mag(raw_data.mag);
  }
#elif defined DBA
  if (boardimu::mpu6500_enabled) {
    mpu_get_data(raw_data.gyro, raw_data.accel, raw_data.mag, raw_data.temp);
  }
#endif  // DBC/DBA
}

// 加热电阻输出端口PWM初始化
void imu::TempControl::init(void) {
  HAL_TIM_Base_Start(htim_);
  HAL_TIM_PWM_Start(htim_, tim_channel_);
}

// IMU加热电阻PWM控温
void imu::TempControl::handle(float temp_ref, float temp_fdb) {
  pwm_ = (uint16_t)math::limitMin(pid_.calc(temp_ref, temp_fdb), 0);
  __HAL_TIM_SetCompare(htim_, tim_channel_, pwm_);
}

// 全部IMU初始化
void imu::initAll(void) {
  // 底盘使用外置imu, 不使用板载imu
  // boardimu::initSensor(board_imu.raw_data_);
}

// 全部IMU处理
void imu::handleAll(void) {
  static bool ext_imu_initialized = false;
  static bool gimbal_imu_initialized = false;
  if (!ext_imu_initialized) {
    ext_imu_initialized =
        ext_imu.initFromAccel(0.f, extimu::param.g_norm_thres);
  }
  if (!gimbal_imu_initialized) {
    gimbal_imu_initialized =
        gimbal_imu.initFromAccel(0.f, gimbalimu::param.g_norm_thres);
  }
  body_imu.update();
  // board_imu.update();
  gimbal_imu.update();
}
