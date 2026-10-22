#ifndef IMU_HPP
#define IMU_HPP

#include <stdint.h>

// Pure attitude processing: right-handed body axes, stationary +Z points upward.
// No STM32, SPI, GPIO, board mounting or scheduler dependencies.
class Imu {
public:
    struct Angles { float roll, pitch, yaw; }; // radians, Z-Y-X convention
    static constexpr uint32_t calibration_samples = 1000; // 2 seconds at 500 Hz

    void reset();
    void update(const float body_acc[3], const float body_gyro[3], float dt);
    bool calibrated() const { return ready_; }
    uint32_t calibrationCount() const { return count_; }
    const float *bias() const { return bias_; }
    const float *acceleration() const { return acc_; }
    const float *angularRate() const { return gyro_; }
    Angles complementary() const;
    Angles mahony() const;

private:
    struct Quaternion { float w = 1, x = 0, y = 0, z = 0; };
    Quaternion comp_, mahony_;
    float acc_[3] = {}, gyro_[3] = {}, bias_[3] = {};
    float mean_g_[3] = {}, mean_a_[3] = {}, m2_g_[3] = {}, m2_a_[3] = {};
    float integral_[3] = {};
    uint32_t count_ = 0;
    bool ready_ = false;

    void clearCalibration();
    void calibrate(const float gyro[3]);
    static Angles angles(const Quaternion &q);
    static Quaternion quaternion(float roll, float pitch, float yaw);
    static void integrate(Quaternion &q, const float gyro[3], float dt);
    static float wrap(float angle);
};

#endif
