#ifndef MPU6050_H_
#define MPU6050_H_

#include <avr/io.h>
#include "mpu6050_registers.h"

#define MPU6050_ADDR (0x68<<1)	//shifted because it will be concatenated with i2c read or write bit (0 or 1)

#define MPU6050_GYRO_FS		MPU6050_GYRO_FS_250		//+-250deg/s fullscale
#define MPU6050_ACCEL_FS	MPU6050_ACCEL_FS_2		//+-2g fullscale

extern void mpu6050_init();
extern void mpu6050_getRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
extern uint8_t mpu6050_testConnection();
extern int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data);
extern void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data);
extern void mpu6050_writeByte(uint8_t regAddr, uint8_t data);
extern int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
extern void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
extern void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);


#endif