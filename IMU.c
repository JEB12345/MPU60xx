/* 
 * File:   IMU.c
 * Author: Jonathan
 *
 * Created on October 23, 2014, 11:26 AM
 *
 */

#include "IMU.h"
#include "../sensor_pindefs.h"
#include "../sensor_can.h"
#include <xc.h>


uint8_t accelRange;
uint8_t gyroRange;

volatile uint8_t i2c_internal_data[16];
volatile return_value_t i2c_internal_ret;

uint8_t IMU_Init(uint32_t i2cFreq, uint32_t sysFreq)
{
	// Initializes the i2c line, MPU60x0, and MAG3110
	I2C_Init(I2C_CALC_BRG(i2cFreq, sysFreq));

	accelRange = ACCEL_FS_2;
	gyroRange = GYRO_FS_250;
        I2C_WriteToReg(MPU60XX_ADDRESS, RA_PWR_MGMT_1, 0b10000000);//Reset

        //add some delay during reset
        Delay_us(1000);
        Delay_us(1000);
        Delay_us(1000);
        Delay_us(1000);
        Delay_us(1000);
        Delay_us(1000);
        Delay_us(1000);

	MPU60xx_Init(accelRange, gyroRange, true);
	MAG3110_Init();

	IMU_SetMPUMaster();

        i2c_1_enable_interrupts();
        i2c_internal_ret = RET_ERROR;
        return 0;
}

void IMU_SetMPUMaster(void)
{
	// Disables i2c aux passthrough and sets the MPU as a master on the aux line
	MPU60xx_SetI2CAuxPassthrough(false);
//
//	// Writes configuration bits for the MPU i2c master
//	// Enables wait for external data for sync, and read characteristics
//	// Also sets the MPU master i2c clock at 400kHz
//	I2C_WriteToReg(MPU60XX_ADDRESS, RA_I2C_MST_CTRL, (0x40 | 0xD));
//
//	// Write the MAG3110 address (0x0E) to the Slave 0 address register
//	// Also sets the Slave 0 read/write bit as a read transaction
//	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SLV0_ADDR, (1 << 7) | MAG3110_ADDRESS);
//
//	// Writes the Address to start reading from as the MAG3110 X MSB register (0x01)
//	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SLV0_REG, MAG_OUT_X_MSB);
//
//	// This register controls how the MPU will retrieve data from Slave 0
//	// This write will enable Slave 0, Set grouping of words to be
//	// Odd then Even (since MAG_OUT_X_MSB = 0x01) and reads 6 bytes
//	I2C_WriteToReg(MPU60XX_ADDRESS, RA_SLV0_CTRL, (0x90 | 0x6));

	// Enables FIFO for Temp, Gyros, Accels, and Slave 0 data
	I2C_WriteToReg(MPU60XX_ADDRESS, RA_FIFO_EN, 0xF8);//0xF9);
}

void IMU_CopyOutput(IMU_Data *imuData, MPU6050_Data *mpuData, MAG3110_Data *magData)
{
    float Q[4];
    IMU_GetQuaternion(Q);

    //TODO: copy one or the other, depending on which algorithm is used.
    CO(ahrs_quaternion_Quaternion_ahrs_1) = Q[0]; 
    CO(ahrs_quaternion_Quaternion_ahrs_2) = Q[1]; 
    CO(ahrs_quaternion_Quaternion_ahrs_3) = Q[2]; 
    CO(ahrs_quaternion_Quaternion_ahrs_4) = Q[3]; 
    
    CO(imu_quaternion_Quaternion_imu_1) = Q[0]; 
    CO(imu_quaternion_Quaternion_imu_2) = Q[1]; 
    CO(imu_quaternion_Quaternion_imu_3) = Q[2]; 
    CO(imu_quaternion_Quaternion_imu_4) = Q[3]; 
    
    CO(accel_accel_x_norm) = imuData->accelX;
    CO(accel_accel_y_norm) = imuData->accelY;
    CO(accel_accel_z_norm) = imuData->accelZ;

    CO(gyro_gyro_x_norm) = imuData->gyroX;
    CO(gyro_gyro_y_norm) = imuData->gyroY;
    CO(gyro_gyro_z_norm) = imuData->gyroZ;

    CO(mag_mag_x_norm) = imuData->magX;
    CO(mag_mag_y_norm) = imuData->magY;
    CO(mag_mag_z_norm) = imuData->magZ;

    CO(accel_accel_x_raw) = mpuData->accelX;
    CO(accel_accel_x_raw) = mpuData->accelY;
    CO(accel_accel_x_raw) = mpuData->accelZ;

    CO(gyro_gyro_x_raw) = mpuData->gyroX;
    CO(gyro_gyro_x_raw) = mpuData->gyroY;
    CO(gyro_gyro_x_raw) = mpuData->gyroZ;

    CO(mag_mag_x_raw) = magData->magX;
    CO(mag_mag_x_raw) = magData->magY;
    CO(mag_mag_x_raw) = magData->magZ;
}

void IMU_CopyI2CData(MPU6050_Data *mpuData, MAG3110_Data *magData)
{
    if (i2c_internal_ret == RET_OK) {
        mpuData->accelX = i2c_internal_data[1];
        mpuData->accelX |= ((uint16_t) i2c_internal_data[0]) << 8;
        mpuData->accelY = i2c_internal_data[3 ];
        mpuData->accelY |= ((uint16_t) i2c_internal_data[2]) << 8;
        mpuData->accelZ = i2c_internal_data[5 ];
        mpuData->accelZ |= ((uint16_t) i2c_internal_data[4]) << 8;
        mpuData->temp = i2c_internal_data[7 ];
        mpuData->temp |= ((uint16_t) i2c_internal_data[6]) << 8;
        mpuData->gyroX = i2c_internal_data[9];
        mpuData->gyroX |= ((uint16_t) i2c_internal_data[8]) << 8;
        mpuData->gyroY = i2c_internal_data[11];
        mpuData->gyroY |= ((uint16_t) i2c_internal_data[10]) << 8;
        mpuData->gyroZ = i2c_internal_data[13 ];
        mpuData->gyroZ |= ((uint16_t) i2c_internal_data[12]) << 8;
    }
}

void IMU_GetData()
{
    if(i2c_internal_ret==RET_OK || i2c_internal_ret==RET_ERROR){
        //start new read
        i2c_internal_ret = RET_UNKNOWN;
//        i2c_1_read(MPU60XX_ADDRESS, RA_FIFO_COUNT_H, 2, i2c_internal_data, &i2c_internal_ret);
        i2c_1_read(MPU60XX_ADDRESS, RA_FIFO_R_W, 14, i2c_internal_data, &i2c_internal_ret);
    }

//    uint16_t bytesInBuffer;
//    static uint8_t i2c_op = 0;
//
//    volatile static  uint8_t i2c_internal_data[16];
//    volatile static return_value_t i2c_internal_ret = RET_ERROR;
//    if (i2c_internal_ret == RET_OK || i2c_internal_ret == RET_ERROR) {
//        if (i2c_op == 0) {
//            //read byte count
//            if (i2c_1_read(MPU60XX_ADDRESS, RA_FIFO_COUNT_H, 2, i2c_internal_data, &i2c_internal_ret) == RET_OK) {
//                //success!
//                i2c_op = 1;
//            } else {
//                //couldn't start an operation, will try again later
//                i2c_internal_ret = RET_ERROR;
//            }
//        } else if (i2c_op == 1) {
//            if (i2c_internal_ret == RET_OK) {
//                //check number of bytes received
//                bytesInBuffer = (i2c_internal_data[0] << 8) | i2c_internal_data[1];
//                if(bytesInBuffer>=14){
//                    //schedule data read
//                    if (i2c_1_read(MPU60XX_ADDRESS, RA_FIFO_R_W, 14, i2c_internal_data, &i2c_internal_ret) == RET_OK) {
//                        //success!
//                        i2c_op = 2;
//                    } else {
//                        //couldn't start an operation, will try again later
//                        i2c_internal_ret = RET_ERROR;
//                        i2c_op = 0;
//                    }
//                }
//            } else {
//                //abort
//                i2c_op = 0;
//            }
//
//        } else if(i2c_op==2){
//            if(i2c_internal_ret == RET_OK){
//                i2c_op = 0;
//                    mpuData->accelX = i2c_internal_data[1];
//                    mpuData->accelX |= ((uint16_t) i2c_internal_data[0]) << 8;
//                    mpuData->accelY = i2c_internal_data[3 ];
//                    mpuData->accelY |= ((uint16_t) i2c_internal_data[2]) << 8;
//                    mpuData->accelZ = i2c_internal_data[5 ];
//                    mpuData->accelZ |= ((uint16_t)i2c_internal_data[4]) << 8;
//                    mpuData->temp = i2c_internal_data[7 ];
//                    mpuData->temp |= ((uint16_t)i2c_internal_data[6]) << 8;
//                    mpuData->gyroX = i2c_internal_data[9];
//                    mpuData->gyroX |= ((uint16_t)i2c_internal_data[8]) << 8;
//                    mpuData->gyroY = i2c_internal_data[11];
//                    mpuData->gyroY |= ((uint16_t)i2c_internal_data[10]) << 8;
//                    mpuData->gyroZ = i2c_internal_data[13 ];
//                    mpuData->gyroZ |= ((uint16_t)i2c_internal_data[12]) << 8;
////                    LED_4 = 0;
//
//            } else {
//                //Abort
//                i2c_op = 0;
//            }
//        }
//
//    } else {
//        //still running, do nothing
//    }

//	uint8_t data[20]; //TODO: Set to proper byte size dependent on what sampling is being done.
//        uint16_t tmp = 0;
//        uint16_t temperature = 0;
////        // Read how many bytes have been loaded into the FIFO
////	I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_CONFIG, data, 1);
////        tmp = data[0];
//
////        I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_TEMP_OUT_H, data, 2);
////        temperature = (data[0]<<8)+data[1];
////        temperature = temperature/340+36;
//
//	// Read how many bytes have been loaded into the FIFO
//	I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_FIFO_COUNT_H, data, 2);
//	uint16_t bytesInBuffer = (data[0] << 8) | data[1];
//
////        I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, 0x3A, data, 1);
////        tmp = data[0];
////        if(tmp&0b1){
////            temperature = 0xFF+tmp;
////
////        }
////        if(bytesInBuffer>=14){
////            LED_4 = 0;
////        }
//
//	// If there's data available, read a single timestamp
//	if (bytesInBuffer >= 14){//sizeof(data)) { //14 without magnetometer, 20 with magnetometer
//		I2C_ReadFromReg_Burst(MPU60XX_ADDRESS, RA_FIFO_R_W, data, 14);////sizeof(data));
//                LED_4 = 0;
//		// Now fit this data into our nice pretty structs
//		mpuData->accelX = data[1];
//		mpuData->accelX |= data[0] << 8;
//		mpuData->accelY = data[3];
//		mpuData->accelY |= data[2] << 8;
//		mpuData->accelZ = data[5];
//		mpuData->accelZ |= data[4] << 8;
//		mpuData->temp = data[7];
//		mpuData->temp |= data[6] << 8;
//		mpuData->gyroX = data[9];
//		mpuData->gyroX |= data[8] << 8;
//		mpuData->gyroY = data[11];
//		mpuData->gyroY |= data[10] << 8;
//		mpuData->gyroZ = data[13];
//		mpuData->gyroZ |= data[12] << 8;
//
//		magData->mag_X_msb = data[14];
//		magData->mag_X_lsb = data[15];
//		magData->magX = (((int16_t) magData->mag_X_msb) << 8) | magData->mag_X_lsb;
//		magData->mag_Y_msb = data[16];
//		magData->mag_Y_lsb = data[17];
//		magData->magY = (((int16_t) magData->mag_Y_msb) << 8) | magData->mag_Y_lsb;
//		magData->mag_Z_msb = data[18];
//		magData->mag_Z_lsb = data[19];
//		magData->magZ = (((int16_t) magData->mag_Z_msb) << 8) | magData->mag_Z_lsb;
//	}
}

void IMU_normalizeData(MPU6050_Data mpuData, MAG3110_Data magData, IMU_Data *normData)
{
	// Derive Normalization Factor
	float accelNormalizer = 16384.0 / (accelRange + 1);
	float gyroNormalizer = 131.0 / (gyroRange + 1);
	const static float magNormalizer = 10.0; // TODO: add support for user defined mag offset

	// Normalize Accel, Gyro, and Mag data
	// Accels are in units of m/s^2
	normData->accelX = (mpuData.accelX / accelNormalizer) * G_FORCE;
	normData->accelY = (mpuData.accelY / accelNormalizer) * G_FORCE;
	normData->accelZ = (mpuData.accelZ / accelNormalizer) * G_FORCE;
	// Gyros are in Degrees/s
	normData->gyroX = (mpuData.gyroX / gyroNormalizer);
	normData->gyroY = (mpuData.gyroY / gyroNormalizer);
	normData->gyroZ = (mpuData.gyroZ / gyroNormalizer);
	// Mags are in micro Teslas
	normData->magX = (magData.magX / magNormalizer);
	normData->magY = (magData.magY / magNormalizer);
	normData->magZ = (magData.magZ / magNormalizer);
}

void IMU_UpdateIMU(const IMU_Data *newData)
{
	// gyro values are expressed in deg/sec, convert to radians/sec
	MadgwickAHRSupdateIMU(DEG2RAD(newData->gyroX), DEG2RAD(newData->gyroY), DEG2RAD(newData->gyroZ), newData->accelX, newData->accelY, newData->accelZ);
}

void IMU_UpdateAHRS(const IMU_Data *newData)
{
	// gyro values are expressed in deg/sec, convert to radians/sec
	MadgwickAHRSupdate(DEG2RAD(newData->gyroX), DEG2RAD(newData->gyroY), DEG2RAD(newData->gyroZ), newData->accelX, newData->accelY, newData->accelZ, newData->magX, newData->magY, newData->magZ);
}

void IMU_GetQuaternion(float q[4])
{
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}
