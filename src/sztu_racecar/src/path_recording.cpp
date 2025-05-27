/* includes ------------------------------------------------------------------*/
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
class PathRecording
{
    public:
        PathRecording();
        ~PathRecording();

    private:
        ros::Timer timer1; /*< 定时器1 */
        ros::NodeHandle n_; /*< 发布和订阅结点服务 */
        ros::Subscriber odom_sub;   /*< 订阅里程计 */
        ros::Publisher path_pub;    /*< 发布路径 */
        ros::Publisher path_pub_end; /*< 录制成后发布规划路径 */

        nav_msgs::Odometry odom; /*< 里程数据 */
        nav_msgs::Path recorded_path;  /*< 录制的路径 */
        nav_msgs::Path optimised_path; /*< 优化的路径 */
        nav_msgs::Path published_path;  /*< 发布的路径 */

        double controller_freq;
        double min_distance; /*< 录制间隔距离 */
        double max_distance; /*< 录制最大距离 */
        double cum_distance; /*< 累计距离 */
        bool is_path_pub; /*< 是否发布路径 */

        /* 订阅里程计回调函数 */
        void odomCB(const nav_msgs::Odometry::ConstPtr& ori_odom);

        /* 根据两点坐标计算距离 */
        double getPtDistance(const geometry_msgs::Point& curPos1,
				             const geometry_msgs::Point& curPos2);
        /* 计算路径 */
        void recordingPath();

        /* 优化路径 */
        void optimisingPath();

        /* 定时器回调函数 */
        void mainLoopCB(const ros::TimerEvent&);
};

/* function ------------------------------------------------------------------*/
/**
  * @brief  构造函数
  * @param  
  * @retval 
  * @attention 初始化参数、订阅、发布
  */
PathRecording::PathRecording()
{
    /* 参数结点 */
    ros::NodeHandle pn("~");
    pn.param("controllerFreq", controller_freq, 100.0); /*< 控制频率 */
    pn.param("minDistance", min_distance, 0.025); /*< 录制间隔距离 */
    pn.param("maxDistance", max_distance, 10.0); /*< 录制最大距离 */

    cum_distance = 0.0; /*< 初始化累计距离 */
    is_path_pub = false;

    /* 订阅者 */
    odom_sub = n_.subscribe("/odometry/filtered", 1,
                                    &PathRecording::odomCB, this);

    /* 发布者 */
    path_pub = n_.advertise<nav_msgs::Path>("/recording/path", 1);
    path_pub_end = n_.advertise<nav_msgs::Path>("/recorded/path", 1);

    recorded_path.header.stamp = ros::Time::now();
    recorded_path.header.frame_id = "odom";

    /* 定时器1 */
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq),
                             &PathRecording::mainLoopCB, this);
}

/**
  * @brief  析构函数
  * @param  
  * @retval 
  * @attention
  */
PathRecording::~PathRecording()
{

}


/**
  * @brief  根据两点坐标计算距离
  * @param  坐标点1, 坐标点2
  * @retval 两点距离
  * @attention
  */
double PathRecording::getPtDistance(const geometry_msgs::Point& curPos1,
				  const geometry_msgs::Point& curPos2)
{
	double dx = curPos1.x - curPos2.x;
	double dy = curPos1.y - curPos2.y;
	double dist = sqrt(dx*dx + dy*dy);
	return dist;
}

/**
  * @brief  里程计回调函数
  * @param  里程计原始数据
  * @retval 
  * @attention 目前里程计仅使用x速度和yaw角度数据
  */
void PathRecording::odomCB(const nav_msgs::Odometry::ConstPtr& ori_odom)
{
    odom = *ori_odom;
}

/**
  * @brief  录制路径
  * @param
  * @retval 
  * @attention 
  *     1.每隔一定距离记录一个点
  *     2.当总路程大于一定值，且回到原点时完成录制并发布路径
  */
void PathRecording::recordingPath()
{
    static geometry_msgs::Point lastPoint = odom.pose.pose.position; /*< 保存上一点的值 */
    double distance = getPtDistance(lastPoint, odom.pose.pose.position);
    
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header.stamp = ros::Time(0);
    pose_tmp.header.frame_id = "odom";
    pose_tmp.pose.position.x = odom.pose.pose.position.x;
    pose_tmp.pose.position.y = odom.pose.pose.position.y;
    pose_tmp.pose.orientation = odom.pose.pose.orientation;

    /* 每隔一定距离记录一个点 */
    if (fabs(distance) > min_distance)
    {
        lastPoint = odom.pose.pose.position;
        recorded_path.poses.push_back(pose_tmp);
    }

    /* 计算累计距离 */
    cum_distance += (1/controller_freq) * odom.twist.twist.linear.x;

    //ROS_INFO("cum_distance: %f", cum_distance);
    /* 达到最大录制距离 */
    if ((fabs(cum_distance) >= max_distance))
    {
        /* 与初始点的距离 */
        distance = getPtDistance(recorded_path.poses[0].pose.position,
                                  odom.pose.pose.position);

        //ROS_INFO("begin_distance: %f", distance);
        /* 接近起点 */
        if (fabs(distance) < 0.3 && !is_path_pub)
        {
            is_path_pub = true; /*< 只发布一次 */
            ROS_INFO("Recording end!");
            published_path = recorded_path;
            path_pub_end.publish(published_path); /*< 发布路径，小车开始跟随 */
        }
    }
}

// void PathRecording::bezierCurve()

/**
  * @brief  优化路径
  * @param
  * @retval 
  * @attention 
  */
void PathRecording::optimisingPath()
{

}

/**
  * @brief  定时器循环执行
  * @param 
  * @retval 
  * @attention 
  */
void PathRecording::mainLoopCB(const ros::TimerEvent&)
{
    recordingPath();
    optimisingPath();
    path_pub.publish(recorded_path);
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
    ros::init(argc, argv, "path_recording");

    PathRecording path_record;

    ros::spin();

    return 0;
}
