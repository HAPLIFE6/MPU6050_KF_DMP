
#include "ZTJS.h"
#include "tim.h" 


#ifdef KF_Calculate

extern uint16_t count_nth;
float v_roll = 0, v_pitch = 0;
/*定义微分时间*/
float now = 0, lasttime = 0, dt = 0; 
float gyro_roll = 0, gyro_pitch = 0 ; 
float acc_roll = 0, acc_pitch = 0; 
float k_roll = 0, k_pitch = 0 ;
float EP[2][2] ={{1,0},{0,1}}; 
float KK[2][2] ={{0,0},{0,0}}; 
float Q[2][2]  ={{0.03,0},{0,0.03}};
float R[2][2]  ={{0.3,0},{0,0.3}};

void Alpha_ZT_Init()
{
    MPU6050_Init();
    HAL_Delay(10);
}
//[ax]=[-sinp]
//[ay]=[cosp*sinr]           *g
//[az]=[cosp*cosr]
/*
void Alpha_ACC_Calculate(float *back_rec)
{   short *pre_accdata;
    float pre_x;float pre_y;float pre_z;
    float row,pitch,yaw;
    MPU6050ReadAcc(pre_accdata);
    pre_x=pre_accdata[0];pre_y=pre_accdata[1];pre_z=pre_accdata[2];
    row=atan((float)pre_y/(float)pre_z)*57.29578;
    pitch=-atan((float)pre_x/(float)((float)sqrt(pow(pre_y,2)+pow(pre_z,2))))*57.29578;
    yaw=0;
    back_rec[0]=row;back_rec[1]=pitch;back_rec[2]=yaw;
}
void Alpha_Gyro_Calculate(float *gyro_rec,float *gyro_v)
{   short *pre_gyrodata;
    float pre_gx;float pre_gy;float pre_gz;
    float dr,dp;
    float dr_v,dp_v,dy_v;
    MPU6050ReadGyro(pre_gyrodata);
    dr=gyro_rec[0];dp=gyro_rec[1];
    pre_gx=(float)pre_gyrodata[0];pre_gy=(float)pre_gyrodata[1];pre_gz=(float)pre_gyrodata[2];
    dr_v=(pre_gx+(((sin(dp)*sin(dr))/cos(dp))*pre_gy)+(((cos(dr)*sin(dp))/cos(dp))*pre_gz));
    dp_v=((cos(dr)*pre_gy)-(sin(dr)*pre_gz));
    dy_v=(((sin(dr)/cos(dp))*pre_gy)+((cos(dr)/cos(dp))*pre_gz));
    gyro_v[0]=dr_v;gyro_v[1]=dp_v;gyro_v[2]=dy_v;
}
*/
uint32_t dt_calculate_start(void)
{
    HAL_TIM_Base_Start_IT(&htim1);
    return 0;
}
uint32_t dt_calculate_end(void)
{
    uint32_t count;
    count=__HAL_TIM_GET_COUNTER(&htim1);
    count=count_nth*1000+count;
    HAL_TIM_Base_Stop(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1,0);
    count_nth=0;
    return count;
}
void KF_process(float *ans)
{   
  dt_calculate_start();
  short ggyro[3];short aacc[3];
  float gyro[3];float acc[3];
  MPU6050ReadGyro(ggyro);
  MPU6050ReadAcc(aacc);
  gyro[0]=ggyro[0];gyro[1]=ggyro[1];gyro[2]=ggyro[2];
  acc[0]=aacc[0];acc[1]=aacc[1];acc[2]=aacc[2];
  //第一步
  v_roll = (gyro[0]) + ((sin(k_pitch)*sin(k_roll))/cos(k_pitch))*(gyro[1]) + ((sin(k_pitch)*cos(k_roll))/cos(k_pitch))*gyro[2];//roll轴的角速度
  v_pitch = cos(k_roll)*(gyro[1]) - sin(k_roll)*gyro[2];
  //v_yaw=(((sin(k_roll)/cos(k_pitch))*gyro[1])+((cos(k_roll)/cos(k_pitch))*gyro[2]));  //add
  gyro_roll = k_roll + dt*v_roll;
  gyro_pitch = k_pitch + dt*v_pitch;

  //gyro_yaw  =k_yaw+dt*v_yaw;  //add
  //第二步
  EP[0][0] = EP[0][0] + Q[0][0];
  EP[0][1] = EP[0][1] + Q[0][1];
  EP[1][0] = EP[1][0] + Q[1][0];
  EP[1][1] = EP[1][1] + Q[1][1];

  //第三步
  KK[0][0] = EP[0][0]/(EP[0][0]+R[0][0]);
  KK[0][1] = 0;
  KK[1][0] = 0;
  KK[1][1] = EP[1][1]/(EP[1][1]+R[1][1]);

  //第四步
  acc_roll = atan((acc[1] ) / (acc[2]))*57.29578;
  acc_pitch = -1*atan((acc[0] ) / sqrt(pow((acc[1] ),2)+ pow((acc[2]),2)))*57.29578;
  k_roll = gyro_roll + KK[0][0]*(acc_roll - gyro_roll);
  k_pitch = gyro_pitch + KK[1][1]*(acc_pitch - gyro_pitch);

  //k_yaw=   gyro_yaw ;
  //第五步
  EP[0][0] = (1 - KK[0][0])*EP[0][0];
  EP[0][1] = 0;
  EP[1][0] = 0;
  EP[1][1] = (1 - KK[1][1])*EP[1][1];

  dt=dt_calculate_end()/1000000; //test
  ans[0]=k_roll;ans[1]=k_pitch;
  //默认航向角YAW为0，因为acc计无法测得yaw，因此yaw无法卡尔曼滤波
}
#endif

#ifdef MPU_Calculate

//暂时未更新

#endif

