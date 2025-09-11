#include "Arduino.h"
#include "LSM6.h"
#include "LIS3MDL.h"
#include "ClusterConfig.h"
#include "angle_utils.h"
#include "Wire.h"



#pragma once
class Gyro {
  public:
    
    struct MagCal {
      float offX   = 0.0f;
      float offY   = 0.0f;
      float offZ   = 0.0f;
      float scaleX = 1.0f;
      float scaleY = 1.0f;
      float scaleZ = 1.0f;
    };
  
    void setMagCal(const MagCal& c){ m_cal = c; }

    bool begin(TwoWire& bus = Wire){
    bus.begin();
    m_lsm6.setBus(&bus);
    m_lis3.setBus(&bus);
    if(!m_lsm6.init(LSM6::device_auto, LSM6::sa0_auto)) return false;
    if(!m_lis3.init(LIS3MDL::device_auto, LIS3MDL::sa1_auto)) return false;
    m_lsm6.enableDefault(); // ±2 g, ±245 dps
    m_lis3.enableDefault(); // ±4 gauss
    calibrateGyroBias(300); // ~0.3s if loop runs at ~1kHz; keeps robot still
    m_lastMicros = micros();
    m_ready = true; return true;
  }

  void update(){
    if(!m_ready) return;
      // dt
    uint32_t now = micros();
    float dt = (now - m_lastMicros) * 1e-6f;
    if (dt <= 0) dt = 1e-3f; // guard
    m_lastMicros = now;

    // read sensors
    m_lsm6.read(); // fill a (accel) and g (gyro) raw
    m_lis3.read(); // fill m (mag) raw

    // raw -> gyro z [rad/s] (for 245 dps full-scale, sensitivity ≈ 8.75 mdps/LSB)
    // source: LSM6DSO datasheet / Pololu LSM6 lib defaults
    const float dps_per_lsb = 8.75e-3f; // deg/s per LSB @245 dps
    float gz = ((float)m_lsm6.g.z - m_gbiasZ) * dps_per_lsb * (PI/180.0f);

    // integrate yaw
    m_yaw = DirectionUtils::wrap2pi(m_yaw + gz * dt);
    // accel-based roll/pitch
    float ax = m_lsm6.a.x, ay = m_lsm6.a.y, az = m_lsm6.a.z;
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    // mag hard/soft iron 
    float mx = (m_lis3.m.x - m_cal.offX) * m_cal.scaleX;
    float my = (m_lis3.m.y - m_cal.offY) * m_cal.scaleY;
    float mz = (m_lis3.m.z - m_cal.offZ) * m_cal.scaleZ;

    // tilt compensation
    float mxh = mx * cosf(pitch) + mz * sinf(pitch);
    float myh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    float hdg = atan2f(-myh, mxh) + m_decl; // magnetic -> true heading
    if (hdg < 0) hdg += 2.0f * PI; if (hdg >= 2.0f * PI) hdg -= 2.0fPI;

    // complementary correction
    const float beta = 0.02f; // magnetometer blend rate
    float err = DirectionUtils::angdiff(hdg, m_yaw);
    m_yaw = DirectionUtils::wrap2pi(m_yaw + beta * err);
  }

  float headingRad() const { return m_yaw; }
  void zeroToCurrent(){ m_yaw = 0.0f; }

// Optional: quick hard-iron calib helper – call while spinning slowly 360° and log min/max externally
  void setMagHardIronFromMinMax(float minX, float maxX, float minY, float maxY, float minZ, float maxZ){
    m_cal.offX = 0.5f * (minX+maxX); m_cal.offY = 0.5f * (minY+maxY); m_cal.offZ = 0.5f * (minZ+maxZ);
    m_cal.scaleX = 2.0f/(maxX-minX); m_cal.scaleY = 2.0f/(maxY-minY); m_cal.scaleZ = 2.0f/(maxZ-minZ);
  }


private:
  void calibrateGyroBias(uint16_t samples){
    long sx=0, sy=0, sz=0;
    for(uint16_t i=0;i<samples;++i){ m_lsm6.readGyro(); sx+=m_lsm6.g.x; sy+=m_lsm6.g.y; sz+=m_lsm6.g.z; delay(1); }
      m_gbiasZ = (float)sz / samples;
    }

  LSM6 m_lsm6;
  LIS3MDL m_lis3;
  uint32_t m_lastMicros = 0;
  bool m_ready=false;
  float m_yaw = 0.0f;
  float m_gbiasZ = 0.0f;
  float m_decl = 0.0f;
  MagCal m_cal{};

};