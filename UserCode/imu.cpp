#include "imu.hpp"
#include <cmath>

namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float g = 9.80665f;
float norm(const float v[3]) { return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
}

void Imu::reset() { *this = Imu{}; }

float Imu::wrap(float a)
{
    while (a > pi) a -= 2*pi;
    while (a < -pi) a += 2*pi;
    return a;
}

Imu::Quaternion Imu::quaternion(float r, float p, float y)
{
    float cr=std::cos(r/2), sr=std::sin(r/2);
    float cp=std::cos(p/2), sp=std::sin(p/2);
    float cy=std::cos(y/2), sy=std::sin(y/2);
    Quaternion q;
    q.w=cr*cp*cy+sr*sp*sy; q.x=sr*cp*cy-cr*sp*sy;
    q.y=cr*sp*cy+sr*cp*sy; q.z=cr*cp*sy-sr*sp*cy;
    return q;
}

Imu::Angles Imu::angles(const Quaternion &q)
{
    return {std::atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
            std::asin(clamp(2*(q.w*q.y-q.z*q.x), -1, 1)),
            std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))};
}

void Imu::integrate(Quaternion &q, const float v[3], float dt)
{
    const Quaternion old=q;
    const float h=0.5f*dt, x=v[0], y=v[1], z=v[2];
    q.w += h*(-old.x*x-old.y*y-old.z*z);
    q.x += h*( old.w*x+old.y*z-old.z*y);
    q.y += h*( old.w*y-old.x*z+old.z*x);
    q.z += h*( old.w*z+old.x*y-old.y*x);
    float n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    q.w/=n; q.x/=n; q.y/=n; q.z/=n;
}

void Imu::clearCalibration()
{
    count_=0;
    for (int i=0; i<3; ++i) mean_g_[i]=mean_a_[i]=m2_g_[i]=m2_a_[i]=0;
}

void Imu::calibrate(const float v[3])
{
    float an=norm(acc_);
    if (std::fabs(an-g)>0.15f*g || norm(v)>0.15f) {
        clearCalibration();
        return;
    }
    ++count_;
    for (int i=0; i<3; ++i) {
        float d=v[i]-mean_g_[i];
        mean_g_[i]+=d/count_; m2_g_[i]+=d*(v[i]-mean_g_[i]);
        d=acc_[i]-mean_a_[i];
        mean_a_[i]+=d/count_; m2_a_[i]+=d*(acc_[i]-mean_a_[i]);
    }
    if (count_ < calibration_samples) return;
    for (int i=0; i<3; ++i) {
        // Reject a moving calibration window even when the average rate is small.
        if (m2_g_[i]/(count_-1)>0.0001f || m2_a_[i]/(count_-1)>0.09f) {
            clearCalibration();
            return;
        }
    }
    for (int i=0; i<3; ++i) bias_[i]=mean_g_[i];
    float r=std::atan2(mean_a_[1], mean_a_[2]);
    float p=std::atan2(-mean_a_[0], std::hypot(mean_a_[1], mean_a_[2]));
    comp_=mahony_=quaternion(r,p,0);
    ready_=true;
}

void Imu::update(const float a[3], const float v[3], float dt)
{
    if (!a || !v || !std::isfinite(dt) || dt<=0 || dt>0.05f) return;
    for (int i=0; i<3; ++i)
        if (!std::isfinite(a[i]) || !std::isfinite(v[i])) return;

    for (int i=0; i<3; ++i) {
        acc_[i]=a[i];
        gyro_[i]=v[i]-bias_[i];
    }
    if (!ready_) {
        calibrate(v);
        for (int i=0; i<3; ++i) gyro_[i]=v[i]-bias_[i];
        return;
    }

    float an=norm(acc_);
    // This norm gate rejects obvious linear acceleration, not every possible motion.
    bool trust=an>0.85f*g && an<1.15f*g;

    // Complementary filter: quaternion gyro prediction + low-frequency tilt correction.
    integrate(comp_,gyro_,dt);
    if (trust) {
        Angles e=angles(comp_);
        float r=std::atan2(acc_[1],acc_[2]);
        float p=std::atan2(-acc_[0],std::hypot(acc_[1],acc_[2]));
        float beta=dt/(0.5f+dt); // alpha=tau/(tau+dt), tau=0.5 s
        // Suppress Euler tilt correction close to pitch singularity; gyro still propagates.
        if (std::fabs(std::cos(e.pitch))>0.1f) {
            e.roll+=beta*wrap(r-e.roll);
            e.pitch+=beta*(p-e.pitch);
            comp_=quaternion(e.roll,e.pitch,e.yaw);
        }
    }

    // Six-axis Mahony: measured up direction CROSS predicted up direction.
    float corrected[3]={gyro_[0],gyro_[1],gyro_[2]};
    if (trust) {
        const Quaternion &q=mahony_;
        float up[3]={2*(q.x*q.z-q.w*q.y), 2*(q.y*q.z+q.w*q.x),
                     1-2*(q.x*q.x+q.y*q.y)};
        float m[3]={acc_[0]/an,acc_[1]/an,acc_[2]/an};
        float error[3]={m[1]*up[2]-m[2]*up[1], m[2]*up[0]-m[0]*up[2],
                        m[0]*up[1]-m[1]*up[0]};
        for (int i=0; i<3; ++i) {
            integral_[i]=clamp(integral_[i]+0.05f*error[i]*dt,-0.05f,0.05f);
            corrected[i]+=2.0f*error[i]+integral_[i];
        }
    }
    integrate(mahony_,corrected,dt);
}

Imu::Angles Imu::complementary() const { return angles(comp_); }
Imu::Angles Imu::mahony() const { return angles(mahony_); }
