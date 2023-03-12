#ifndef __BSP_ZTJS_H

#include "main.h"
#include "bsp_mpu6050.h"
#include "math.h"

#define KF_Calculate  //define KF_Calculate 是使用卡尔曼滤波算法   define MPU_Calculate 是使用DMP姿态解算





void KF_process(float *ans);
#define __BSP_ZTJS_H
#endif
