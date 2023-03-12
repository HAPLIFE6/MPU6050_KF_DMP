
#include "bsp_MPU6050.h"


void MPU6050_WriteReg(uint8_t reg,uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,&data,1,1000);
}
void MPU6050_WriteManyData(uint8_t reg,unsigned char *data,uint8_t num)
{
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,data,num,1000);
}

void MPU6050_ReadData(uint8_t reg,uint8_t *Read,uint8_t num)
{
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,Read,num,1000);
}

void MPU6050_Init(void)
{
	HAL_Delay(100);
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	   
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);    
	HAL_Delay(200);
	

}

uint8_t MPU6050ReadID(void)
{
	uint8_t Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    
	return Re;	
}

void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}
void MPU6050ReadTemp(short *tempData)
{
	uint8_t buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);
    *tempData = (buf[0] << 8) | buf[1];
}
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);
    temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;
}
//以下是为了适配DMP库不得不改的函数名

uint8_t  MPU_IIC_Init(void)
{
    MPU6050_Init();
    return 0;
}
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    MPU6050_ReadData(reg,buf,len);
    return 0;
}
uint8_t  MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    MPU6050_WriteManyData(reg,buf,len);
    return 0;
}
void delay_ms(uint16_t time_ms)
{
    HAL_Delay(time_ms);
}
