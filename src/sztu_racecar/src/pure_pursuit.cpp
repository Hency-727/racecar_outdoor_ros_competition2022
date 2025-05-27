#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <vector>
#include "PID.h"
#include "sensor_msgs/LaserScan.h"
#include <sys/time.h>
#define SIMULATION 0
#include <time.h>
#define PI 3.14159265358979
#define MAX_ANGLE 80   //75
#define MAX_PWM 1700
#define MIN_PWM 1400
#define PID_LIMIT 500

/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
    public:
        PurePursuit();
        void initMarker();
        
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
	    //use for calculate Lfw
        double getPtDistance(const geometry_msgs::Point& curPt1, const geometry_msgs::Point& curPt2);
        bool isNextPtAwayFromCurPt(const geometry_msgs::Point& nextPos, const geometry_msgs::Point& curPos, const double& distTsd);
        double getL1Distance(const double& currentVcmd, double cur);

        //use for calcurlate SteerAngle
        double getEta(const geometry_msgs::Point& odomCar2PtVec);
        double getSteeringAngle(double eta);
        double getYawFromPose(const geometry_msgs::Pose& carPose);    
        geometry_msgs::Point getOdomCar2PtVec(const geometry_msgs::Point& carPos, const geometry_msgs::Point& point, const double& carPose_yaw);    
        double getSteeringAngleFromPath(const geometry_msgs::Pose& carPose, const double& distPt2PtTsd, const double& distCar2PtTsd);

        double getCur(const geometry_msgs::Point& pathPt1, const geometry_msgs::Point& pathPt2, const geometry_msgs::Point& pathPt3);
        double getCurFromPath(const geometry_msgs::Point& carPosition, const double& distPt2PtTsd, const double& distCar2Pt);

        //use for calculate PID
        double posPIDCal(struct PID *pp, double thisError);
        void initPID();
    
    private:
        struct PID motorPID_;
        struct PID tebPID_, steerPID_;
	clock_t startTime;
        ros::NodeHandle n_;
        ros::Subscriber odom_sub_, path_sub_;
        ros::Publisher pub_, marker_pub_;
        ros::Timer timer1_;
        tf::TransformListener tf_listener_;

        ros::Subscriber err_sub_;
        ros::Subscriber scan_err_sub_;
        ros::Publisher cmd_vel_pub_;

        visualization_msgs::Marker points_, line_strip_, goal_circle_, cur_points_, u_points_;
        geometry_msgs::Twist cmdVel_;
        nav_msgs::Odometry odom_;
        nav_msgs::Path orimapPath_;
        geometry_msgs::Twist cmd_vel_;

        float err;
        float scan_err;

        double L_, Lfw_, Vcmd_, setV_, lfw_, dt_, controllerFreq_, kappi_, UcurveSpeed_, CommonTurnSpeed_;
        double LfwDecGain_, VcmdDecGain_, baseAngle_, turnRadius_, goalRadius_, dPhiDecGain_, angleGain_, tebAngle_;

        double motorProportionL_, motorIntegralL_,motorProportionH_, motorIntegralH_,speedErrorTsd_;
        double steerProportionL_, steerDifferentialL_, steerProportionH_, steerDifferentialH_;

        double curDecGain_, distCar2PtTsd4Kappi_, distPt2PtTsd4Kappi_;
        double distCar2PtTsd4U_, distPt2PtTsd4U_, reduceSpdTsd_, distCar2PtTsd4Common_;
        double distCar2PtTsd4Lfw_, distPt2PtTsd4Lfw_,distCar2PtTsd4LfwMin_;
        double lengthUCurve_;
        

        int baseSpeed_;
        int erasePointCnt_, goalReachedCnt_, goalNum_, goalReceivedCnt_;
        int goalCount_, goalReachFlag_;
        double inUCurveCount_, outUCurveCount_, smallYawCount_;
        bool foundForwardPt_, goalReceived_, goalReached_, carStop_, reduceSpd_, pathReceived_, pathKeep_, globalPathReceived_;
        bool isUCurveBegin_, isUCurveEnd_, isInUCurve_,isInUCurveJudge_;
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void controlLoopCB(const ros::TimerEvent&);
        void errCB(const std_msgs::Float32::ConstPtr& msg);
        void scanErrCB(const std_msgs::Float32::ConstPtr& msg);
        
        int eraseGlobalPointCnt_;
        double distPt2PtTsd4Common_;
        bool isInUCurveEndTest_;
        bool isInCommonCurve_, isInCommonCurveEndTest_;
        double lengthTestUCur_, lengthTestUCurOut_, lengthForceUCurOut_, lengthUcureEnd_, lengthTestNotUCur_, length4Trajetory_,length4TrajetoryTestUcurve_, lengthTestNotUCurOut_, length4TrajetoryTest90Obst_ ; 
        double lengthTestCommon_, lengthTestCommonOut_, lengthForceCommonOut_, lengthCommonEnd_;
        double reduceSpdTsd4Common_;
        bool brakeForce_, LfwForce_;
        double LfwForceLength_;
        double finalLength_, finalVcmd_;
};


PurePursuit::PurePursuit()
{
    //Private parameters handler
    ros::NodeHandle pn("~");
    startTime=ros::Time::now().toNSec();
    //Car parameter
    pn.param("L", L_, 0.38);     //车轴距离                          
    pn.param("Vcmd", Vcmd_, 2.0);    //给定速度
    pn.param("lfw", lfw_, 0.17);     //
    pn.param("goalRadius", goalRadius_, 0.5);    //目标半径
    pn.param("turnRadius", turnRadius_, 1.0);    //转弯半径
    pn.param("UcurveSpeed", UcurveSpeed_, 2.3);    
    pn.param("UcurveSpeed", CommonTurnSpeed_, 2.5);    
    
    //Controller parameter
    pn.param("controllerFreq", controllerFreq_, 20.0);   //控制频率
    pn.param("baseSpeed", baseSpeed_, 1560);     //前进死区
    pn.param("baseAngle", baseAngle_, 90.0);     //舵机中值
    pn.param("VcmdDecGain", VcmdDecGain_, 0.9);  //速度减益
    pn.param("LfwDecGain", LfwDecGain_, 0.25);   //前视距离减益
    pn.param("dPhiDecGain", dPhiDecGain_, 0.9);  //角度补偿减益
    pn.param("angleGain", angleGain_, 2.0);  //角度补偿减益
    
    pn.param("motorProportionL", motorProportionL_, 20.0);     
    pn.param("motorIntegralL", motorIntegralL_, 0.6);
    pn.param("motorProportionH", motorProportionH_, 23.0);     
    pn.param("motorIntegralH", motorIntegralH_, 0.8);
    pn.param("speedErrorTsd", speedErrorTsd_, 0.8);
    pn.param("steerProportionL", steerProportionL_, 3.0);
    pn.param("steerDifferentialL", steerDifferentialL_, 10.0);
    pn.param("steerProportionH", steerProportionH_, 3.0);
    pn.param("steerDifferentialH", steerDifferentialH_, 10.0);

	pn.param("curDecGain", curDecGain_, 0.2);    //前视距离曲率减益
	pn.param("distCar2PtTsd4Lfw", distCar2PtTsd4Lfw_, 1.4);   //车到点的距离阈值
	pn.param("distPt2PtTsd4Lfw", distPt2PtTsd4Lfw_, 0.2);        //点到点的距离阈值
    pn.param("distCar2PtTsd4LfwMin", distCar2PtTsd4LfwMin_, 0.6);
    
	pn.param("distCar2PtTsd4Kappi", distCar2PtTsd4Kappi_, 0.6);   //车到点的距离阈值
	pn.param("distPt2PtTsd4Kappi", distPt2PtTsd4Kappi_, 0.2);        //点到点的距离阈值

    pn.param("distCar2PtTsd4U", distCar2PtTsd4U_, 0.2);
    pn.param("distCar2PtTsd4Common", distCar2PtTsd4Common_, 0.5);
 
    pn.param("distPt2PtTsd4U",distPt2PtTsd4U_, 1.4);       //点到点的距离阈值
    pn.param("distPt2PtTsd4Common",distPt2PtTsd4Common_, 0.4);       //点到点的距离阈值
	pn.param("reduceSpdTsd", reduceSpdTsd_, 0.7);    //减速的曲率阈值
    pn.param("reduceSpdTsd4Common", reduceSpdTsd4Common_, 0.8);
    pn.param("goalNum", goalNum_, 4);  
    pn.param("length4TrajetoryTestUcurve", length4TrajetoryTestUcurve_, 30.0);
    pn.param("length4TrajetoryTest90Obst", length4TrajetoryTest90Obst_, 0.0);

    pn.param("finalLength", finalLength_, 0.0);
    pn.param("finalVcmd", finalVcmd_, 0.0);
    //Publishers and Subscribers
    odom_sub_ = n_.subscribe("/odometry/filtered", 1, &PurePursuit::odomCB, this);
    path_sub_ = n_.subscribe("/recorded/path", 1, &PurePursuit::pathCB, this);

    pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);

    cmd_vel_.linear.x = 0;
    cmd_vel_.angular.z = 90;
    err_sub_ = n_.subscribe("Vision_Angle", 1, &PurePursuit::errCB, this);
    scan_err_sub_ = n_.subscribe("/scan/bias", 1, &PurePursuit::scanErrCB, this);
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);

    err = 0.0;
    scan_err = 0.0;
    //Timer
    timer1_ = n_.createTimer(ros::Duration((1.0)/controllerFreq_), &PurePursuit::controlLoopCB, this); // Duration(0.05) -> 20Hz
    Lfw_ = 0.3;                  //前视距离
    goalCount_ = 0;              //目标点计数
    goalReachFlag_ = 0;          //目标点到达计数
    dt_ = 1.0/controllerFreq_;    //控制周期
    foundForwardPt_ = false;     //是否发现预瞄点
    pathReceived_ = false;       //是否收到路径
    globalPathReceived_ = false; //是否收到路径
    pathKeep_ =  false;          //pathKeep_为true表示地图未更新　地图若未更新再一次controlCB时从mapPath.poses[Pointcnt]开始算
    goalReceived_ = false;       //是否接受到目标
    goalReached_ = false;        //是否到达目标
    reduceSpd_ = false;          //是否减速
    carStop_ = true;             //是否停车
    
    isInUCurveEndTest_ = isInCommonCurve_ = isInCommonCurveEndTest_ = false;
    cmdVel_.linear.x = 1500;     //1500 for stop
    cmdVel_.angular.z = baseAngle_;   //舵机中值
    setV_ = Vcmd_;
    lengthUCurve_ = 0.0;           //U弯长度
    inUCurveCount_ = 0;            //进U弯标志位计数
    outUCurveCount_ = 0;           //出U弯标志位计数 
    smallYawCount_ = 0;
    
    isUCurveBegin_ = isUCurveEnd_ = isInUCurve_ = isInUCurveJudge_ = false;
    goalReceivedCnt_ = 0;
    erasePointCnt_ = 0;
    eraseGlobalPointCnt_ = 0;
    goalReachedCnt_ = 0;
    length4Trajetory_ = 0;
    LfwForceLength_ = 0;
    brakeForce_ = LfwForce_ = false;
    //Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed_);
    ROS_INFO("[param] baseAngle: %f", baseAngle_);
    ROS_INFO("[param] Vcmd: %f", Vcmd_);
    ROS_INFO("[param] Lfw: %f", Lfw_);

    initMarker(); 
}



void PurePursuit::errCB(const std_msgs::Float32::ConstPtr& msg)
{
    err = msg->data;
    static float old_err = 0;

    float derr = err - old_err;   
    cmd_vel_.linear.x = Vcmd_*1.1;   //1.1
    cmd_vel_.angular.z = 90 - err*0.5 - derr*0.0;   //0.5  //越小越偏蓝越大越敏感
	clock_t endTime;
	
 endTime=ros::Time::now().toNSec();
double timeuse=(endTime-this->startTime)/10e6;
if(3280<timeuse)   //3630
{
    cmd_vel_.linear.x = 1500;
    cmd_vel_.angular.z = 1500;
	// this->cmdVel_.linear.x = 0;
        //this->cmdVel_.angular.z = 0;
        //cmd_vel_pub_.publish(cmd_vel_);
}
ROS_INFO("The Time for using: %lf ms",timeuse);

if(timeuse)
    old_err = err; 
//    ROS_INFO("ERR: %f", err);
}

void PurePursuit::scanErrCB(const std_msgs::Float32::ConstPtr& msg)
{
    scan_err = msg->data;
}

//可视化目标点、Lfw
void PurePursuit::initMarker()
{
    points_.header.frame_id = u_points_.header.frame_id = cur_points_.header.frame_id = line_strip_.header.frame_id = goal_circle_.header.frame_id = "odom";
    points_.ns = u_points_.ns = cur_points_.ns = line_strip_.ns = goal_circle_.ns = "Markers";
    points_.action = u_points_.action = cur_points_.action = line_strip_.action = goal_circle_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w = u_points_.pose.orientation.w = cur_points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_circle_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    goal_circle_.id = 2;
    cur_points_.id = 3;
    u_points_.id = 4;

    points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle_.type = visualization_msgs::Marker::CYLINDER;
    cur_points_.type = visualization_msgs::Marker::POINTS;
    u_points_.type = visualization_msgs::Marker::POINTS;

    //show the point of U curve
    u_points_.scale.x = 0.2;
    u_points_.scale.y = 0.2;

    //show the point of the curve
    cur_points_.scale.x = 0.3;
    cur_points_.scale.y = 0.3;

    // POINTS markers use x and y scale for width/height respectively
    points_.scale.x = 0.2;
    points_.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip_.scale.x = 0.1;

    goal_circle_.scale.x = goalRadius_;
    goal_circle_.scale.y = goalRadius_;
    goal_circle_.scale.z = 0.1;


    //u curve points are red
    u_points_.color.r = 1.0;
    u_points_.color.g = 0.0;
    u_points_.color.b = 0.0;
    u_points_.color.a = 1.0;

    //cur points are yellow
    cur_points_.color.r = 1.0;
    cur_points_.color.g = 1.0;
    cur_points_.color.b = 0.0;
    cur_points_.color.a = 0.5;

    // Points are green
    points_.color.g = 1.0f;
    points_.color.a = 1.0;

    // Line strip is blue
    line_strip_.color.b = 1.0;
    line_strip_.color.a = 1.0;

    //goal_circle_ is yellow
    goal_circle_.color.r = 1.0;
    goal_circle_.color.g = 1.0;
    goal_circle_.color.b = 0.0;
    goal_circle_.color.a = 0.5;
}

//PID初始化
void PurePursuit::initPID()
{
    motorPID_.PID_Out = 0;
    motorPID_.Error = 0;
    motorPID_.LastError = 0;
    motorPID_.Proportion = motorProportionL_;
    motorPID_.Integral = motorIntegralL_;

    steerPID_.PID_Out = 0;
    steerPID_.Error = 0;
    steerPID_.LastError = 0;
    steerPID_.Proportion = steerProportionL_;
    steerPID_.Differential = steerDifferentialL_;
    
    tebPID_.PID_Out = 0;
    tebPID_.Error = 0;
    tebPID_.LastError = 0;
    tebPID_.Proportion = steerProportionH_;
    tebPID_.Differential = steerDifferentialH_;
}

//位置式PID
double PurePursuit::posPIDCal(struct PID *pp, double thisError)
{
    pp->LastError = pp->Error;
    pp->Error = thisError;
    double PID_Out = pp->Proportion * pp->Error + pp->Differential * (pp->Error - pp->LastError);
    if(PID_Out > MAX_ANGLE)
    {
        PID_Out = MAX_ANGLE;
    }
    else if(PID_Out < -MAX_ANGLE)
    {
        PID_Out = -MAX_ANGLE;  
    }
    pp->PID_Out = PID_Out;
    return PID_Out;    
}




void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    static double cum_distance;
    odom_ = *odomMsg; 
    if (pathReceived_)
    {
      cum_distance += (1/50) * odom_.twist.twist.linear.x;

        if (cum_distance > 7.0)
        {
            double car2GoalDist = getPtDistance(orimapPath_.poses[0].pose.position,
                                             odom_.pose.pose.position);
            if (car2GoalDist < 0.5)
            {	
                goalReached_ = true;
            }
        }
        
    }
}




void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    orimapPath_ = *pathMsg;

    this->pathReceived_ = true;
    this->goalReceived_ = true;
    this->pathKeep_ = false; //pathKeep_为false表示地图更新；
    this->erasePointCnt_ = 0;
}

//得到点到车的偏航角
double PurePursuit::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp, yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp, tmp, yaw);

    return yaw;
}


//旋转向量得到小车的预瞄点  carPos：小车此时的坐标   point：小车预瞄点的坐标   carPose_yaw：欲转角
geometry_msgs::Point PurePursuit::getOdomCar2PtVec(const geometry_msgs::Point& carPos, const geometry_msgs::Point& point, const double& carPose_yaw)
{
    geometry_msgs::Point odomCar2PtVec;
    odomCar2PtVec.x = cos(carPose_yaw)*(point.x - carPos.x) + sin(carPose_yaw)*(point.y - carPos.y);
    odomCar2PtVec.y = -sin(carPose_yaw)*(point.x - carPos.x) + cos(carPose_yaw)*(point.y - carPos.y);
    return odomCar2PtVec;
}

//两点求距离
double PurePursuit::getPtDistance(const geometry_msgs::Point& curPos1, const geometry_msgs::Point& curPos2)
{
	double dx = curPos1.x - curPos2.x;
	double dy = curPos1.y - curPos2.y;
	double dist = sqrt(dx*dx + dy*dy);
	return dist;
}


//三点求曲率
double PurePursuit::getCur(const geometry_msgs::Point& pathPt1, const geometry_msgs::Point& pathPt2, const geometry_msgs::Point& pathPt3)
{
	double k1, k2;
	if(pathPt1.x == pathPt2.x && pathPt3.x == pathPt2.x) 
        return 0.0;

	else if((float)(pathPt2.y - pathPt1.y)/(float)(pathPt2.x - pathPt1.x) ==
		    (float)(pathPt2.y - pathPt3.y)/(float)(pathPt2.x - pathPt3.x))
		return 0.0;
	else
    {
		double dista, distb, distc, cosA, sinA, cur;
		dista = getPtDistance(pathPt2, pathPt1);
		distb = getPtDistance(pathPt2, pathPt3);
		distc = getPtDistance(pathPt3, pathPt1);
 		cosA = (distb*distb + distc*distc - dista*dista)/(2*distb*distc);
		sinA = sqrt(1 - cosA*cosA);
		cur = 2*sinA/dista; 
		return cur;
	}
}

//从路径中得到曲率
//param: carPosition:   车辆当前位置
//       distPt2PtTsd:  点到点距离阈值
//       distCar2PtTsd: 车到第一个点的距离阈值

double PurePursuit::getCurFromPath(const geometry_msgs::Point& carPosition, 
                                 const double& distPt2PtTsd, 
                                 const double& distCar2PtTsd)
{
    geometry_msgs::Point Pt1 = carPosition;
    geometry_msgs::Point Pt2 = carPosition;
    geometry_msgs::Point Pt3 = carPosition;
    double cur = 0.0;

	for(int i = this->eraseGlobalPointCnt_; i < this->orimapPath_.poses.size(); i++)
   	{

    	geometry_msgs::PoseStamped mapPath_pose = this->orimapPath_.poses[i];
   		geometry_msgs::PoseStamped odomPath_pose;

        try
        {

            tf_listener_.transformPose("odom", ros::Time(0) , mapPath_pose, "map" ,odomPath_pose);
            geometry_msgs::Point odomPath_wayPt = odomPath_pose.pose.position;
			bool _isCurPtAwayFromCarDist = isNextPtAwayFromCurPt(odomPath_wayPt, carPosition, distCar2PtTsd);

			if(_isCurPtAwayFromCarDist)
			{
		   		Pt1 = this->orimapPath_.poses[i].pose.position;
                		this->eraseGlobalPointCnt_ = i;
				bool _isCurPtAwayFromTheNext = false;

		    	for(; i < this->orimapPath_.poses.size(); i++)
		    	{
				_isCurPtAwayFromTheNext = isNextPtAwayFromCurPt(this->orimapPath_.poses[i].pose.position, Pt1, distPt2PtTsd);

					if(_isCurPtAwayFromTheNext)
					{
						Pt2 = this->orimapPath_.poses[i].pose.position;
						_isCurPtAwayFromTheNext = false;
						break;
					}

					else if(!_isCurPtAwayFromTheNext && i == this->orimapPath_.poses.size()-1)
					{
						cur = -1.0;
					}
		    	}

			for(; i < this->orimapPath_.poses.size();i++)
		    	{
				_isCurPtAwayFromTheNext = isNextPtAwayFromCurPt(this->orimapPath_.poses[i].pose.position, Pt2, distPt2PtTsd);

					if(_isCurPtAwayFromTheNext)
					{
						Pt3 = this->orimapPath_.poses[i].pose.position;
						_isCurPtAwayFromTheNext = false;
						break;
					}

					else if(!_isCurPtAwayFromTheNext && i == this->orimapPath_.poses.size()-1)
					{
						cur = -1.0;
					}
		    	}
		    	break;
			}
	   	}

        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
        }
	}
    if(cur != -1.0)
		cur = getCur(Pt1, Pt2, Pt3); 
	if(cur <= 0.0) 
		cur = 0.0;

    return cur;
}


//判断大于一定阈值的点是否存在
bool PurePursuit::isNextPtAwayFromCurPt(const geometry_msgs::Point& nextPos, 
                                         const geometry_msgs::Point& curPos, 
                                         const double& distTsd)
{
    double dist = getPtDistance(nextPos, curPos);
    if(dist < distTsd)
        return false;
    else if(dist >= distTsd)
        return true;
}



//根据曲率以及当前速度得到Lfw

double PurePursuit::getL1Distance(const double& currentVcmd, double cur)
{
	double dec = 0;
    double L1 = 0;

	dec =  fabs(cur) * this->curDecGain_;

    if (dec >= 0.9)
    {
        dec = 0.9;
    }

	L1 = (this->LfwDecGain_ * currentVcmd  + this->turnRadius_) * (1 - dec);

    if(L1 >= 5.0)
    {
        L1 =5.0; 
    }
    return L1;
}

bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}


//从路径中三个点求取最合适的舵机打角 车的位置，欲瞄点的位置，还有周边障碍物的位置
double PurePursuit::getSteeringAngleFromPath(const geometry_msgs::Pose& carPose, 
                                             const double& distPt2PtTsd, 
                                             const double& distCar2PtTsd)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);

    geometry_msgs::Point Pt1 = carPose_pos;
    geometry_msgs::Point Pt2 = carPose_pos;
    geometry_msgs::Point Pt3 = carPose_pos;
    geometry_msgs::Point NearestPt = carPose_pos;
    double steeringAngle = 0.0;

    bool _firstPointFlag, _secondPointFlag, _thirdPointFlag, _nearestPointFlag;
    _nearestPointFlag = _firstPointFlag = _secondPointFlag = _thirdPointFlag = false;

    for(int i = this->erasePointCnt_; i < orimapPath_.poses.size(); i++)
    //for(int i = 0; i < orimapPath_.poses.size(); i++)
    {
        geometry_msgs::PoseStamped mapPath_pose = orimapPath_.poses[i];
        geometry_msgs::PoseStamped odomPath_pose;
        
	try
        {
            tf_listener_.transformPose("odom", ros::Time(0) , mapPath_pose, "map" ,odomPath_pose);
            geometry_msgs::Point odomPath_wayPt = odomPath_pose.pose.position;
 
            bool _isCurPtAwayFromCarDist = isNextPtAwayFromCurPt(odomPath_wayPt, carPose_pos,  this->distCar2PtTsd4LfwMin_);    

	        bool _isForwardWayPt = isForwardWayPt(odomPath_wayPt,carPose);
			if(_isCurPtAwayFromCarDist && _nearestPointFlag==false && _isForwardWayPt)
			{
		   	    NearestPt = odomPath_wayPt;
                _nearestPointFlag = true;
                this->erasePointCnt_ = i;   //若路径未更新，从上一次的预瞄点位置开始寻找新的预瞄点
                //ROS_INFO("-----%d---%.2f----",i,this->distCar2PtTsd4LfwMin_);
            }
       	        
	        _isCurPtAwayFromCarDist = isNextPtAwayFromCurPt(odomPath_wayPt, carPose_pos, distCar2PtTsd);
	        _isForwardWayPt = isForwardWayPt(odomPath_wayPt,carPose);
			if(_isCurPtAwayFromCarDist && _firstPointFlag==false && _isForwardWayPt)
			{
		   	    Pt1 = odomPath_wayPt;
                _firstPointFlag = true;
                //this->erasePointCnt_ = i;   //若路径未更新，从上一次的预瞄点位置开始寻找新的预瞄点
            }
            if(_firstPointFlag && _secondPointFlag == false)
            {
				bool _isCurPtAwayFromTheNext = isNextPtAwayFromCurPt(odomPath_wayPt, Pt1, distPt2PtTsd);
				if(_isCurPtAwayFromTheNext)
				{
					Pt2 = odomPath_wayPt;
					_isCurPtAwayFromTheNext = false;
                    _secondPointFlag = true;
				}
				else if(!_isCurPtAwayFromTheNext && i == orimapPath_.poses.size()-1)
				{
					Pt2 = Pt1;
                    _secondPointFlag = true;
                    _isCurPtAwayFromTheNext = false;
				}
            }
            if(_secondPointFlag && _thirdPointFlag == false)
            {
				bool _isCurPtAwayFromTheNext = isNextPtAwayFromCurPt(odomPath_wayPt, Pt2, distPt2PtTsd);
				if(_isCurPtAwayFromTheNext)
				{
					Pt3 = odomPath_wayPt;
					_isCurPtAwayFromTheNext = false;
                    _thirdPointFlag = true;
                    break;
				}
				else if(!_isCurPtAwayFromTheNext && i == orimapPath_.poses.size()-1)
				{
					Pt3 = Pt2;
                    _thirdPointFlag = true;
                    _isCurPtAwayFromTheNext = false;
                    break;
				}
            }
           
	   	}
 	    catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
        }
	}
	
	double eta0 = getEta(getOdomCar2PtVec(carPose_pos, NearestPt, carPose_yaw));   //车的位置，预瞄点的位置，还有周边障碍物的位置
    double steeringAngle0 = getSteeringAngle(eta0);
	
    double eta1 = getEta(getOdomCar2PtVec(carPose_pos, Pt1, carPose_yaw));
    double steeringAngle1 = getSteeringAngle(eta1);

    double eta2 = getEta(getOdomCar2PtVec(carPose_pos, Pt2, carPose_yaw));
    double steeringAngle2 = getSteeringAngle(eta2);

    double eta3 = getEta(getOdomCar2PtVec(carPose_pos, Pt3, carPose_yaw));
    double steeringAngle3 = getSteeringAngle(eta3);
    
    steeringAngle = steeringAngle1;
    if((steeringAngle1>0 && steeringAngle2>0 && steeringAngle3>0)
        ||(steeringAngle1<0 && steeringAngle2<0 && steeringAngle3<0))
    {
        if(std::fabs(steeringAngle) <= std::fabs(steeringAngle2))
            steeringAngle = steeringAngle2;
        if(std::fabs(steeringAngle) <= std::fabs(steeringAngle3))
            steeringAngle = steeringAngle3;
    }
    else
    {
         steeringAngle = steeringAngle2;
    }
    
    if(steeringAngle0 * steeringAngle < 0 && 
       this->isInUCurve_ && 
       (this->lengthForceUCurOut_ < 1.7) )
    {
        if( (this->length4Trajetory_ > length4TrajetoryTest90Obst_) &&
       (this->length4Trajetory_ < length4TrajetoryTest90Obst_ + 5))
       {
         steeringAngle = steeringAngle0 * 3;
       }

         //ROS_INFO("1:%.2f, 2:%.2f, 3:%.2f, steering:%.2f", steeringAngle1, steeringAngle2, steeringAngle3, steeringAngle);
    }  

    return steeringAngle;
}




//求预瞄点与车的夹角
double PurePursuit::getEta(const geometry_msgs::Point& odomCar2PtVec)
{
    return atan2(odomCar2PtVec.y, odomCar2PtVec.x);
}



//求前轮角度
double PurePursuit::getSteeringAngle(double eta)
{
    return -atan2((this->L_*sin(eta)),(this->Lfw_/2+this->lfw_*cos(eta)))*(180.0/PI);
}










//
//定时器中断控制函数
void PurePursuit::controlLoopCB(const ros::TimerEvent&)
{	
 clock_t endTime=ros::Time::now().toNSec();
double timeuse=(endTime-this->startTime)/10e6;
//ROS_INFO("The Time for using: %lf ms",timeuse);
    geometry_msgs::Pose carPose = this->odom_.pose.pose;
    geometry_msgs::Twist carVel = this->odom_.twist.twist;
    this->cmdVel_.linear.x = 0;

    //求曲率
    this->kappi_ = getCurFromPath(carPose.position, this->distPt2PtTsd4Kappi_, this->distCar2PtTsd4Kappi_);

    if(this->pathReceived_ && !this->goalReached_)
    {
        /*Estimate Steering Angle*/
        this->length4Trajetory_ += 1 / this->controllerFreq_ * this->odom_.twist.twist.linear.x;
         
        this->Lfw_ = getL1Distance(carVel.linear.x, this->kappi_);

        double steeringAngle = getSteeringAngleFromPath(carPose, this->distPt2PtTsd4Lfw_, this->Lfw_);
        this->pathKeep_ = true;
	    //ROS_INFO("HELLO*************************************");
        double dPhi = (this->dPhiDecGain_ * carVel.linear.x * this->dt_ * sin(steeringAngle*PI/180) / this->L_)*180/PI; 
        double temp =  posPIDCal(&steerPID_, (steeringAngle + dPhi));
        if(std::fabs(temp) < 20) temp = 0.05 * temp * temp * temp / std::fabs(temp);
        this->cmdVel_.angular.z = this->baseAngle_ - temp + scan_err;
        cmdVel_.angular.x = this->cmdVel_.angular.z*PI/180.0;
   
        ros::NodeHandle n;
        n.getParam("Vcmd", this->Vcmd_);
              
        this->cmdVel_.linear.x = this->Vcmd_*1.45 ;
//cmd_vel_pub_.publish(cmd_vel_);
	    this->pub_.publish(this->cmdVel_);   
    }






    else if (!this->pathReceived_ && !this->goalReached_)
    {

        cmd_vel_pub_.publish(cmd_vel_);
    }



    else if (this->goalReached_)
    {
        this->cmdVel_.linear.x = 0;
        this->cmdVel_.angular.z = 0;
        cmd_vel_pub_.publish(cmd_vel_);
//	    this->pub_.publish(this->cmdVel_);
    }
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "pure_pursuit");
	
    PurePursuit controller;

    controller.initPID();
	
    ros::spin();



    return 0;
}

