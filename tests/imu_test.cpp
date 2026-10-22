#include "imu.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
constexpr float dt=.002f, g=9.80665f, pi=3.14159265359f;
int checks=0;
void check(bool ok, const char *name) {
    ++checks;
    if (!ok) { std::fprintf(stderr,"FAIL: %s\n",name); std::exit(1); }
}
float diff(float x,float y) { return std::atan2(std::sin(x-y),std::cos(x-y)); }
// Ideal body acceleration and angular rate in the public interface.
void sample(Imu &imu, float r,float p,const float w[3],const float bias[3]) {
    float a[3]={-g*std::sin(p),g*std::sin(r)*std::cos(p),g*std::cos(r)*std::cos(p)};
    float v[3]={w[0]+bias[0],w[1]+bias[1],w[2]+bias[2]};
    imu.update(a,v,dt);
}
void calibrate(Imu &imu,const float b[3]) {
    float w[3]={};
    for (unsigned i=0;i<Imu::calibration_samples;++i) sample(imu,0,0,w,b);
    check(imu.calibrated(),"stationary calibration finishes");
}
}
int main() {
    const float b[3]={.012f,-.008f,.006f}, zero[3]={};
    Imu imu;
    for (int i=0;i<500;++i) sample(imu,0,0,zero,b);
    float moving[3]={.3f,0,0}; sample(imu,0,0,moving,b);
    check(imu.calibrationCount()==0,"motion restarts calibration");
    calibrate(imu,b);
    for (int i=0;i<3;++i) check(std::fabs(imu.bias()[i]-b[i])<1e-6f,"bias recovered in body axes");
    for (int i=0;i<5000;++i) sample(imu,0,0,zero,b);
    check(std::fabs(imu.complementary().yaw)<1e-5f,"bias correction prevents stationary yaw drift");
    // Combined roll/pitch/yaw motion verifies body-rate integration and feedback sign.
    for (int i=1;i<=6000;++i) {
        float t=i*dt, r=.6f*std::sin(.7f*t), p=.4f*std::sin(.9f*t), y=.5f*t;
        float rd=.42f*std::cos(.7f*t), pd=.36f*std::cos(.9f*t);
        float w[3]={rd-.5f*std::sin(p),pd*std::cos(r)+.5f*std::sin(r)*std::cos(p),
                    -pd*std::sin(r)+.5f*std::cos(r)*std::cos(p)};
        sample(imu,r,p,w,b);
        auto c=imu.complementary(), m=imu.mahony();
        check(std::fabs(diff(c.roll,r))<.015f && std::fabs(c.pitch-p)<.015f && std::fabs(diff(c.yaw,y))<.02f,
              "complementary tracks coupled rotations including yaw wrap");
        check(std::fabs(diff(m.roll,r))<.015f && std::fabs(m.pitch-p)<.015f && std::fabs(diff(m.yaw,y))<.02f,
              "Mahony tracks coupled rotations including yaw wrap");
    }
    auto before=imu.complementary();
    float bad[3]={std::numeric_limits<float>::quiet_NaN(),0,0};
    imu.update(bad,zero,dt); imu.update(zero,zero,0); imu.update(zero,zero,1);
    check(imu.complementary().roll==before.roll,"invalid sample rejected");
    imu.reset();calibrate(imu,zero);
    // Residual gyro bias: acceleration feedback must pull tilt back toward level.
    float residual[3]={.02f,-.01f,0};
    for(int i=0;i<10000;++i) sample(imu,0,0,residual,zero);
    check(std::fabs(imu.complementary().roll)<.015f,"complementary tilt feedback converges");
    check(std::fabs(imu.mahony().roll)<.012f,"Mahony tilt feedback converges");
    std::printf("PASS: %d checks (calibration, coupled rotations, wrapping, feedback, invalid input)\n",checks);
}
