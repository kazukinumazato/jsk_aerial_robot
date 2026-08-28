/*
******************************************************************************
* File Name          : estimator.h
* Description        : super  class for attitude estiamte algorithm
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __RADXA_ATTITUDE_ESTIMATOR_H
#define __RADXA_ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <radxa/control/math/AP_Math.h>
#include <array>

#ifndef SIMULATION
#include "flashmemory/flashmemory.h"
#define DELTA_T 0.001f
#endif

class RadxaEstimatorAlgorithm
{

public:

  RadxaEstimatorAlgorithm():
    acc_(), gyro_(), mag_(),
    est_g_(), est_m_(),
    mag_dec_valid_(false)
  {
    rot_.identity();
    mag_declination_ = 0;

#ifdef SIMULATION
    prev_time = -1;
#else
    FlashMemory::addValue(&mag_declination_, sizeof(float));
#endif
    mag_dec_rot_.from_euler(0, 0, -mag_declination_); // the mag declination has positive value in WC direction, which is opposite with euler frame
  };

  ~RadxaEstimatorAlgorithm(){}

  void update(const radxa_ap::Vector3f& gyro, const radxa_ap::Vector3f& acc, const radxa_ap::Vector3f& mag)
  {
    /* the sensor data in body frame */
    acc_ = acc;
    gyro_ = gyro;
    mag_ = mag;

    estimation();
  }

  virtual void estimation()
  {
#ifdef SIMULATION
    if(prev_time < 0) DELTA_T = 0;
    else DELTA_T = ros::Time::now().toSec() - prev_time;
    prev_time = ros::Time::now().toSec();
#endif
  };

  radxa_ap::Vector3f getAngular(){return gyro_;}
  radxa_ap::Vector3f getAcc(){return acc_;}
  radxa_ap::Vector3f getMag(){return mag_;}
  radxa_ap::Vector3f getEstG(){return est_g_;}
  radxa_ap::Vector3f getEstM(){return est_m_;}
  radxa_ap::Matrix3f getRotation(){return rot_;}

  virtual radxa_ap::Quaternion getQuaternion() = 0;

#ifndef SIMULATION
  bool getMagDecValid() { return mag_dec_valid_; }
  float getMagDeclination() { return mag_declination_;}
  void setMagDeclination(float mag_dec)
  {
    mag_declination_ = mag_dec;
    FlashMemory::erase();
    FlashMemory::write();
    mag_dec_valid_ = true;
  }
#endif

protected:
  radxa_ap::Vector3f acc_, gyro_, mag_;
  radxa_ap::Vector3f est_g_; /* estimated vetor of gravity */
  radxa_ap::Vector3f est_m_; /* estimated vetor of magnet */
  radxa_ap::Matrix3f rot_;

  /* mag declination */
  bool mag_dec_valid_;
  float mag_declination_;
  radxa_ap::Matrix3f mag_dec_rot_;
#ifdef SIMULATION
  float DELTA_T;
  double prev_time;
#endif
};

#endif

