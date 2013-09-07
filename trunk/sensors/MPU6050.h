#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

// call initI2C before this
int init_MPU6050(void);

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
int read_MPU6050(short*data);	

#ifdef __cplusplus
}
#endif

#endif
