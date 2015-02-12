// a very simple AHRS module exported from MultiWii.
// there is no gyro bias estimation in this code, you need to do gyro bias compensating/estimating outside this module.
// recommended frame of reference:
// 

#pragma once

#include "common/vector.h"

extern float roll;
extern float pitch;
extern float yaw_mag;
extern float yaw_gyro;

extern vector accel_ef;
void update(vector gyro, vector accel, vector mag);