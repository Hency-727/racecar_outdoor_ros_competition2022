/**
  ******************************************************************************
  * @file    wheel_odom_pub.cpp
  * @author  Brosy
  * @brief   发布编码器数据
  * @date    2021-11-7
  ******************************************************************************
  * @attention 
  *
  * Copyright (c) Brosy.
  * All rights reserved.
  *
  * This software component is licensed by Brosy under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *修改记录：
  *<时间>      |<版本>      |<作者>      |<描述>     
  *2021-11-07  |v1.0        |Brosy       |首次发布
  ******************************************************************************
**/
/* includes ------------------------------------------------------------------*/
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <stdint.h>


/* typedef -------------------------------------------------------------------*/
typedef struct mCircle_
{
    int32_t circle;
    int16_t angle;
}M_CIRCLE_T;

/* define --------------------------------------------------------------------*/
#define CIRCLE_FASTED (333)
#define FREQ (100)
/* variables -----------------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
class WheelOdom
{
    public:
        WheelOdom();
        ~WheelOdom();

        double getLinearX();
        void mainLoop();
        void circleContinue(M_CIRCLE_T *mc, uint16_t angle);

    private:
        ros::NodeHandle nh; /*< 发布和订阅结点服务 */
        ros::Publisher wheel_odom_pub;  /*< 发布里程 */

        nav_msgs::Odometry odom_data;
        
        serial::Serial wheel_odom_serial;
        
        std::string serial_port;
        int serial_baud;
        double length;

        M_CIRCLE_T wheel_odom_mc_;
};
/* function ------------------------------------------------------------------*/
/**
  * @brief  构造函数
  * @param  
  * @retval 
  * @attention 初始化参数、订阅、发布
  */
WheelOdom::WheelOdom()
{
    /* 参数结点 */
    ros::NodeHandle pn("~");
    pn.param<std::string>("SerialPort", serial_port, "/dev/wheel_odom");
    pn.param<int>("SerialBaud", serial_baud, 38400);   //38400
    pn.param<double>("Length", length, 0.15707963);
    /* 发布者 */
    wheel_odom_pub = nh.advertise<nav_msgs::Odometry>("/wheel_odom", 1);

    /* 开启串口 */
    serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
    wheel_odom_serial.setPort(serial_port);
    wheel_odom_serial.setBaudrate(serial_baud);
    wheel_odom_serial.setTimeout(time_out);

    try
    {
        wheel_odom_serial.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }

    if(wheel_odom_serial.isOpen())
	{
		ROS_INFO_STREAM("/dev/wheel_odom is opened.");
	}
	else
	{
	}

    wheel_odom_mc_.angle = 0;
    wheel_odom_mc_.circle = 0;
}

WheelOdom::~WheelOdom()
{

}


uint8_t send_buf[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
int32_t old_pos = 0;
double WheelOdom::getLinearX()
{
    
    uint8_t receive_buf[8];
    int32_t now_pos = 0;
    double linear_x = 0;

    wheel_odom_serial.write(send_buf, 8);
    wheel_odom_serial.read(receive_buf, 8-1);

    if (receive_buf[0] == 0x01 && receive_buf[1] == 0x03)
    {
        circleContinue(&wheel_odom_mc_, receive_buf[3] << 8 | receive_buf[4]);
    }

    now_pos = wheel_odom_mc_.circle*1024 + wheel_odom_mc_.angle;
    //ROS_INFO("NOW: %d", now_pos);

    /* v = wr */
    linear_x = length*((double)(now_pos - old_pos)*FREQ)/1024.0;
    old_pos = now_pos;
    //linear_x = speed_w;
    //ROS_INFO("LINEAR_X: %f", linear_x);
    return linear_x*1.3;
}

double cum = 0.0;
void WheelOdom::mainLoop()
{
    odom_data.twist.twist.linear.x = getLinearX();
    cum += 0.01 * odom_data.twist.twist.linear.x;
    ROS_INFO("cum_dis: %f", cum);
    odom_data.header.stamp = ros::Time::now();
    odom_data.header.frame_id = "/wheel_odom"; 
    wheel_odom_pub.publish(odom_data);
}

void WheelOdom::circleContinue(M_CIRCLE_T *mc, uint16_t angle)
{
    if ((angle < CIRCLE_FASTED) && (mc->angle > 1024 - CIRCLE_FASTED))
    {
        mc->circle++;
    }
    else if ((angle > 1024 - CIRCLE_FASTED) && (mc->angle < CIRCLE_FASTED))
    {
        mc->circle--;
    }
    mc->angle = angle;
}

/**
  * @brief  主函数入口
  * @param 
  * @retval 
  * @attention 
  */
int main(int argc, char **argv)
{
    /* 初始化ROS结点 */
    ros::init(argc, argv, "wheel_odom_node");

    WheelOdom odom;
    
    ROS_INFO("wheel_odom_init!");

    ros::Rate loop_rate(FREQ);
    while (ros::ok())
    {
        odom.mainLoop();
        loop_rate.sleep();
    }

    return 0;
}
