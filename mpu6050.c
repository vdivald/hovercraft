#define F_CPU 16000000UL

#include "mpu6050.h"
#include "twi.h"
#include <util/delay.h>

volatile uint8_t buffer[14];

void mpu6050_init()
{
	_delay_ms(100);
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);	//set sleep disabled
	_delay_ms(10);	//delay need to wake up
	
	//set clock source
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
	//set sampe rate
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 4); //1khz / (1 + 4) = 200Hz
	//set gyro range
	mpu6050_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
	//set accel range
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);
}

int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data)	//by mpu6050 product specification page 36/52
{
	uint8_t i = 0;
	int8_t count = 0;

	if(length > 0)
	{
		//request register
		TWIStart();
		TWIWrite(MPU6050_ADDR | I2C_WRITE);
		TWIWrite(regAddr);
		//		_delay_us(10);
		//read data
		TWIStart();
		TWIWrite(MPU6050_ADDR | I2C_READ);
		for(i=0; i<length; i++)
		{
			count++;
			if(i==length-1)
			data[i]=TWIReadNACK();
			else
			data[i]=TWIReadACK();
		}
		TWIStop();
	}
	return count;
}

int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data)
{
	return mpu6050_readBytes(regAddr, 1, data);
}

void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data)
{
	uint8_t i;
	
	if(length > 0)
	{
		//request register
		TWIStart();
		TWIWrite(MPU6050_ADDR | I2C_WRITE);
		TWIWrite(regAddr);
		_delay_us(10);
		//write data
		for(i=0; i<length; i++)
		{
			TWIWrite(data[i]);
		}
	}
	TWIStop();
}

void mpu6050_writeByte(uint8_t regAddr, uint8_t data)
{
	return mpu6050_writeBytes(regAddr, 1, &data);
}

int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	int8_t count = 0;
	
	if(length > 0)
	{
		uint8_t b;
		if ((count = mpu6050_readByte(regAddr, &b)) != 0)
		{
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
	}
	return count;
}

int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
	uint8_t b;
	uint8_t count = mpu6050_readByte(regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	if(length > 0)
	{
		uint8_t b = 0;
		if (mpu6050_readByte(regAddr, &b) != 0)	 //get current data
		{
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			mpu6050_writeByte(regAddr, b);
		}
	}
}

void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	mpu6050_readByte(regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	mpu6050_writeByte(regAddr, b);
}

uint8_t mpu6050_testConnection() 
{
	mpu6050_readBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (uint8_t *)buffer);
	if(buffer[0] == 0x34)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void mpu6050_getRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buffer);

	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}