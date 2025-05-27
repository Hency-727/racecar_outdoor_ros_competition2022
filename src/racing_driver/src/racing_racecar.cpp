#include "../include/racing_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
double Vcmd;
double Speed;
double angle;
double now_speed;

typedef struct PID              // PID结构体定义
{                               
    double  Proportion;         // Proportion   比例系数
    double  Integral;           // Integral     积分系数
    double  Differential;       // Differential 微分系数

    double  PreError;           // Error[-2]    前两拍误差
    double  LastError;          // Error[-1]    前一拍误差
    double  Error;              // Error        误差
    double  cumErro;

    double  PID_Out;            // PID_PWM      PID输出

}PID;

struct PID motorPID_;

//位置式PID
double posPIDCal(struct PID *pp, double thisError)
{
    pp->LastError = pp->Error;
    pp->Error = thisError;
    pp->cumErro += thisError;
    pp->cumErro = pp->cumErro > 100? 100:pp->cumErro;
    pp->cumErro = pp->cumErro < -100? -100:pp->cumErro;

    double PID_Out = pp->Proportion * pp->Error
                    + pp->Differential * (pp->Error - pp->LastError)
                    + pp->cumErro*pp->Integral;

    pp->PID_Out = PID_Out;
    return PID_Out;    
}

void odomCB(const nav_msgs::Odometry& odom)
{
    now_speed = odom.twist.twist.linear.x;
}

void TwistCallback(const geometry_msgs::Twist& twist)
{
    angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;

Speed = twist.linear.x*0.4 + 0.6*twist.linear.x*(1-fabs(angle-1500)/1000);
//ROS_INFO("ANGLE:%f",angle);
//Speed = twist.linear.x;
}

void timer1LoopCB(const ros::TimerEvent&)
{

    int pid_out = posPIDCal(&motorPID_, Speed - now_speed);
    pid_out = pid_out < -1000? -1000 : pid_out;
    pid_out = pid_out > 1000? 1000 : pid_out;

   static int pid_out_lpf; 






pid_out_lpf = 0.02*pid_out + 0.98*pid_out_lpf;   
int pid_out_lpf_ = pid_out_lpf<0? pid_out_lpf-80 : pid_out_lpf; 
send_cmd(uint16_t(1500 + pid_out_lpf_),uint16_t(angle));
//    ROS_INFO("pid_ou: %d", pid_out);
}

void pidInit()
{
    motorPID_.PID_Out = 0;
    motorPID_.Error = 0;
    motorPID_.LastError = 0;
    motorPID_.Proportion = 53;  //50


    motorPID_.Integral = 0.58;  //0.5
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);//38400
    double controller_freq = 100;

    pidInit();

    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/car/cmd_vel",1,TwistCallback);
    ros::Subscriber odom_sub = n.subscribe("/wheel_odom",1,odomCB);
    ros::Timer timer1 =  n.createTimer(ros::Duration((1.0)/controller_freq),
                             &timer1LoopCB);

    ros::spin();
    ROS_INFO("ART_DRIVER STOP*******************************************");
    /* Stop */
    Vcmd = 1500;
    angle = 1500;
    send_cmd(uint16_t(Vcmd),uint16_t(angle));
}
