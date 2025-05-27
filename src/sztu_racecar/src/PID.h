
#ifndef PID_H
#define	PID_H

/***********************************/
typedef struct PID              // PID结构体定义
{                               
    double  Proportion;         // Proportion   比例系数
    double  Integral;           // Integral     积分系数
    double  Differential;       // Differential 微分系数

    double  PreError;           // Error[-2]    前两拍误差
    double  LastError;          // Error[-1]    前一拍误差
    double  Error;              // Error        误差

    double  PID_Out;            // PID_PWM      PID输出

}PID;

#endif


