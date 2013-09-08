#ifndef __HMC5883_H__
#define __HMC5883_H__

#ifdef __cplusplus
extern "C" {
#endif

// call initI2C() and init_MPU6050() before this
int init_HMC5883(void);
int read_HMC5883(short*data);

#ifdef __cplusplus
}
#endif

#endif