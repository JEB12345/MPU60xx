/* 
 * File:   IMU.h
 * Author: Jonathan
 *
 * Created on October 23, 2014, 11:27 AM
 *
 * Any code below which has the following comment around it is derived from the
 * PIC24 FreeIMU project:
 * ****************** FreeIMU ******************************
 * *********************************************************
 */

#ifndef IMU_H
#define	IMU_H

#include "I2CdsPIC.h"
#include "MPU60xx.h"
#include "MAG3110.h"
#include <math.h>

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
} IMU_Data;

#define G_FORCE 9.80665

/* ****************** FreeIMU ******************************
 * ********************************************************* */
#define M_PI 3.1415927f
#define DEG2RAD(d)   (((d)*M_PI)/180.0f)
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain
#define FREQ_HZ 100 // This is half the sample period from sensors
/* ****************** FreeIMU ******************************
 * ********************************************************* */


/**
 * This function will setup the MPU60x0 and the MAG3110 as a single i2c device.
 * The MPU will be set as an i2c master to the MAG3110. This allows the host device
 * to get all 9 DoFs from a single burst read to the MPU.
 *
 * @param i2cFreq The frequency of the i2c line. Should always be 400000 at the moment
 * @param sysFreq This is the system clock of the host device
 */
void IMU_Init(uint32_t i2cFreq, uint32_t sysFreq);

/**
 * Sets up the MPU60x0 device to read data registers for the MAG3110 in a burst fasion (hopefully)
 */
void IMU_SetMPUMaster(void);

/**
 * This function uses burst reading on i2c to read all IMU data and then stores it into their respective structs.
 *
 * @param mpuData MPU6050_Data struct pointer
 * @param magData MAG3110_Data struct pointer
 */
void IMU_GetData(MPU6050_Data *mpuData, MAG3110_Data *magData);

/**
 * Normalizes the raw bit data into their respective units
 * Accel: meters per second squared
 * Gyro: degrees per second
 * Mags: micro teslas
 * 
 * @param mpuData MPU6050_Data datatype which holds the Accel and Gyro data
 * @param magData MAG3110_Data datatype which holds the Mag data
 * @param normData IMU_Data this holds the normalized output data
 */
void IMU_normalizeData(MPU6050_Data mpuData, MAG3110_Data magData, IMU_Data *normData);


/* ****************** FreeIMU ******************************
 * ********************************************************* */
void imu_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void imu_getQ(const IMU_Data *imuData, float *q);
void imu_getEuler(const IMU_Data *imuData, float *angles);
void imu_getYawPitchRoll(const IMU_Data *imuData, float *ypr);
float imu_invSqrt(float number);
/* ****************** FreeIMU ******************************
 * ********************************************************* */
#endif	/* IMU_H */
