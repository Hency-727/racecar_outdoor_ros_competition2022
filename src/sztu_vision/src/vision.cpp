// Copyright 2025 Hency
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <queue>
#include <numeric>
#include <sys/time.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#define debug
#define pi 3.1415926 

int flag = 0;   //当前小车状态是否正常行驶（沿赛道逆时针旋转），如果是5则表示判定成功，小车沿赛道逆时针，flag=-5则是小车沿赛道顺时针

int count = 0;  //每隔15个周期进行一次flag判定，越短判定越快

float save = 0; //下一周期，的小车result 结果缓存

float F = 300;        //摄像内参

double min_count=40;   //红蓝锥桶 识别出的红蓝轮廓拟合的最下垂直矩形的点个数（不是矩形面积是矩形周长）

float acc_angle=36.0f;//每次旋转的最小值（也可以理解为小车在匀速旋转，这个是旋转的角速度）

enum block_color{Blue, Red}; //enum枚举 红蓝锥桶


//对摄像头获取画面进行预处理，处理完成的结果是一个二值图，白色区域为识别出来的区域
cv::Mat preHandle(block_color detec_color, cv::Mat src);             

//根据预处理的红蓝两个二值图，进行锥桶检测，返回最终小车要转的角度
float detection(cv::Mat red_src, cv::Mat blue_src, cv::Mat src);

//对包含锥桶轮廓的最小垂直矩形按照面积进行降序排序
void sortArea(std::vector<cv::Rect> &srcRects);                 

//计算矩形的中心点位置
cv::Point getCenter(cv::Rect rect); 

//识别图像中红蓝锥桶进行角度解算
float Angle_solve (std::vector<cv::Rect> redBlocks, std::vector<cv::Rect> blueBlocks,  cv::Mat src);     

//给定红蓝锥桶b_rect r_rect然后取他的高(通过矩形的高来估计出锥桶离小车距离)，再取红蓝矩形的中心来计算角度
float cen_angle_solve(cv::Rect r_rect, cv::Rect b_rect, cv::Point r_bottom_cen, cv::Point b_bottom_cen); 



int main(int argc, char** argv){
	ros::init(argc, argv, "vision_detector");
	ros::NodeHandle nh;

	std_msgs::Float32 msg;

	cv::VideoCapture camera("/dev/video0");        //打开摄像头

	ros::Publisher pub = nh.advertise<std_msgs::Float32>("Vision_Angle", 1);//将小车要旋转的角度，从Vision_Angle话题发布出去


	ros::Rate r(1000);

	while(ros::ok()){

		cv::Mat frame;
		camera >> frame;              //从摄像头中获取画面
		//cv::flip(frame, frame, -1); //顺时针跑则去掉注释，颠倒视图

		#ifdef debug
		struct timeval t1, t2;      //每帧时间测算，用于计算帧率
		double timeuse;
		gettimeofday(&t1, NULL);
		#endif

		//一、预处理
		cv::Mat red_bi = preHandle((block_color)Red, frame);  //预处理得到，红色锥桶为白色其余为黑的二值图
		cv::Mat blue_bi = preHandle((block_color)Blue, frame);//预处理得到，蓝色锥桶为白色其余为黑的二值图
		

		#ifdef debug
		//显示
		cv::imshow("red", red_bi);
		cv::imshow("blue", blue_bi);
		cv::waitKey(1);
		#endif

		float angle = detection(red_bi, blue_bi, frame);

		#ifdef debug

		gettimeofday(&t2, NULL);
		timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;

		std::cout << "FPS:" << 1/timeuse << std::endl;  // 帧率
		#endif

		msg.data = angle;
		pub.publish(msg); //发送根据图像识别到角度

		ros::spinOnce();  //消息订阅函数，该行代码存疑
	}
	return 0;
}

//图像预处理
/*
block_color detec_color ：预处理是针对红色锥桶还是蓝色锥桶
 cv::Mat src：原图像
*/
cv::Mat preHandle(block_color detec_color, cv::Mat src){
    //1、数据定义
	cv::Mat HSV;
	cv::Mat dst;
	std::vector<cv::Mat> hsvSplit;


	//2、对图像亮度进行均衡化减少误差
	cv::cvtColor(src, HSV, cv::COLOR_BGR2HSV);  //转化成HSV色彩空间

	cv::split(HSV, hsvSplit);                   //将HSV分别拆分成HSV放在hsvSplit

	cv::equalizeHist(hsvSplit[2], hsvSplit[2]); //针对V进行均衡化，降低光照对图像识别的影响
	cv::merge(hsvSplit, HSV);                   //将hsvSplit 中的三个通道H,S,V 融合保存在HSV中

	//3、声明定义，HSV色彩空间的最低阈值与最高阈值，并执行inRange操作对多通道双阈值化操作
	double low_H;
	double high_H;
	double low_S;
	double high_S;
	double low_V;
	double high_V;

	if(detec_color == (block_color)Red){ //红色锥桶的最低与最高阈值
		cv::Mat temp1;
		cv::Mat temp2;

		double low_H1 = 0;
		double high_H1 = 10;
		double low_H2 = 156;
		double high_H2 = 180;
		double low_S2=100;
		low_S = 150;
		high_S = 255;
		low_V = 0;
		high_V = 255;

		cv::inRange(HSV, cv::Scalar(low_H1, low_S, low_V), cv::Scalar(high_H1, high_S, high_V), temp1); //主要是将在两个阈值内的像素值设置为白色（255），而不在阈值区间内的像素值设置为黑色（0）
		cv::inRange(HSV, cv::Scalar(low_H2, low_S2, low_V), cv::Scalar(high_H2, high_S, high_V), temp2);

		dst = temp1+temp2;
	}
	else{                               //蓝色锥桶的最低与最高阈值
		low_H = 103;
		high_H = 120;
		low_S = 131;
		high_S = 255;
		low_V = 0;
		high_V = 255;
		cv::inRange(HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), dst);
	}
    //4、对二值化图进行去噪处理
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  //定义一个5*5的矩形卷积核
	cv::morphologyEx(dst, dst, cv::MORPH_OPEN, element);                          //开操作，先腐蚀再碰撞，去掉黑色区域中的白色噪点
	cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, element);                         //闭操作，先碰撞再腐蚀，去掉白色区域中的黑色噪点

	return dst; //返回二值化结果蓝色/红色区域为白色其余为黑色
}
/*
二、cv::Rect的成员和属性
成员
    rect.tl()//矩形左上角点的坐标
    rect.br()//矩形右下角点的坐标
    rect.size()//矩形的大小
    rect.area()//矩形的面积
    rect.empty() //判断矩形是否为空
    rect.contains() //判断一个点是否在矩形区域内

属性
    rect.x; //表示左上角点x的坐标
    rect.y; //表示左上角点y的坐标
    rect.width; //表示矩形的宽度
    rect.height; //表示矩形的高度
*/
//识别图像中红蓝锥桶
float detection(cv::Mat red_src, cv::Mat blue_src, cv::Mat src){
    //1、声明
	std::vector< std::vector<cv::Point> > redContours; //保存红色的轮廓的二维数组
	std::vector< std::vector<cv::Point> > blueContours;//保存蓝色的轮廓的二维数组
	std::vector<cv::Vec4i> hierarchy;

	std::vector<cv::Rect> redBlock;
	std::vector<cv::Rect> blueBlock;
    //2、获取轮廓
	cv::findContours(red_src, redContours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);  //针对红色锥桶进行轮廓提取，结果保存在redContours
	cv::findContours(blue_src, blueContours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);//针对红色锥桶进行轮廓提取，结果保存在blueContours

    //3、遍历红蓝轮廓，并保存符合条件的轮廓拟合的垂直边界矩形
	for(int i = 0; i < redContours.size(); i++){
		cv::Rect temp = cv::boundingRect(redContours[i]);//计算红色锥桶轮廓的垂直边界最小矩形，矩形是与图像上下边界平行的，返回一个Rect变量
		int height = temp.height;
		int width = temp.width;

    //当锥桶识别到的轮廓的点数量，大于min_count(误导不是面积是点的数量)且长宽比大于0.8小于5则认为这是目标红色锥桶
		if(redContours[i].size() > min_count && (height / width < 5 && height / width > 0.8)){
			redBlock.push_back(temp);
		}
	}
	for(int i = 0; i < blueContours.size(); i++){
		cv::Rect temp = cv::boundingRect(blueContours[i]);
		int height = temp.height;
		int width = temp.width;

		if(blueContours[i].size() > min_count && (height / width < 5 && height / width > 0.8)){
			blueBlock.push_back(temp);
		}
	}

    //4、把轮廓画出来
	#ifdef debug
	for(int i = 0; i < redBlock.size(); i++){
		cv::rectangle(src, redBlock[i].tl(), redBlock[i].br(), cv::Scalar(0, 0, 255), 2, 8, 0);
	}
	for(int i = 0; i < blueBlock.size(); i++){
		cv::rectangle(src, blueBlock[i].tl(), blueBlock[i].br(), cv::Scalar(255, 0, 0), 2, 8, 0);
	}
	#endif

    //5、根据红蓝识别出的垂直边界矩形容器，进行角度解算，src是没用的，结果保存在result中并返回
	float result = Angle_solve(redBlock, blueBlock, src);

	#ifdef debug
	cv::imshow("result", src);
	cv::waitKey(1);
	#endif

	return result;
}

//计算道路中心转向角度
float cen_angle_solve(cv::Rect r_rect, cv::Rect b_rect, cv::Point r_bottom_cen, cv::Point b_bottom_cen){
	float d1;
	float d2;

	float theta = atan(abs(float(r_bottom_cen.x - 320))/F) + atan(abs(float(b_bottom_cen.x - 320))/F);
	if(flag == 5){
		d1 = 0.6 * F / r_rect.height;//高度来反应距离
		d2 = 0.6 * F / b_rect.height;
		return atan(float(r_bottom_cen.x - 320) / F)*(500 / pi) + acos(float(d1 + d2 * cos(theta)) / sqrt(float(d1 * d1 + d2 * d2 + 2*d1*d2*cos(theta))))*(500 / pi);
	}

	else if(flag < 5 && flag > -5){
		return float(r_bottom_cen.x - 320) / F + float(b_bottom_cen.x - 320) / F;
	}

	else{
		d1 = 0.6 * F / b_rect.height;
		d2 = 0.6 * F / r_rect.height;
		return atan(float(b_bottom_cen.x - 320) / F) * (500 / pi) + acos(float(d1 + d2 * cos(theta)) / sqrt(float(d1 * d1 + d2 * d2 + 2*d1*d2*cos(theta)))) * (500 / pi);
	}
}

//计算底边中心
cv::Point getCenter(cv::Rect rect){
	cv::Point cpt;
    	cpt.x = rect.x + int(rect.width/2.0);//向下取整
    	cpt.y = rect.y + int(rect.height/2.0);//向下取整
    	return cpt;
}

//按面积排序矩形
void sortArea(std::vector<cv::Rect> &srcRects){
	cv::Rect tmp;
	int sizeV = srcRects.size();
	for(int i = 1; i < sizeV; i++){
		for(int j = sizeV - 1; j >= i; j--){
			if(srcRects[j].height > srcRects[j - 1].height){
				tmp = srcRects[j-1];
				srcRects[j - 1] = srcRects[j];
				srcRects[j] = tmp;
			}
		}
	}
}

//解算角度
float Angle_solve(std::vector<cv::Rect> redBlocks, std::vector<cv::Rect> blueBlocks, cv::Mat src){
	float result = 0.0f;
	if(count<15) //识别轮数，识别15轮才进行一次flag判定，数值越大反应越慢
	count++;
    //1、对所有识别到的矩形进行降序排序
	sortArea(redBlocks);//对包含红色锥桶轮廓的最小垂直矩形按照面积进行降序排序
	sortArea(blueBlocks);//对包含蓝色锥桶轮廓的最小垂直矩形按照面积进行降序排序

	std::vector<float> center_angles;
	std::vector<cv::Point> r_bottom_cen;
	std::vector<cv::Point> b_bottom_cen;
	float r_x_mean;
	float b_x_mean;
	float sum = 0.0f;

    //2、将矩形容器转换成，保存矩形中心的容器，并算出红蓝矩形的，在水平方向上的平均值
	for(int i = 0; i < redBlocks.size() ; i++){
		r_bottom_cen.push_back(getCenter(redBlocks[i]));//将红色矩形的中心点坐标都保存在r_bottom_cen中
		sum += r_bottom_cen[i].x;
	}
	r_x_mean = sum/r_bottom_cen.size();//保存所有红色矩形（识别的锥桶）的，在水平上的平均值

	sum = 0;
	for(int i = 0; i < blueBlocks.size(); i++){
		b_bottom_cen.push_back(getCenter(blueBlocks[i]));
		sum += b_bottom_cen[i].x;
	}
	b_x_mean = sum/r_bottom_cen.size();


//3、判定是否开始机动，b_x_mean > r_x_mean这种情况是正常情况，判定正常flag+1
	if(flag<5&&flag>-5&&count==15){
		if(b_x_mean > r_x_mean){
			flag++;
		}
		else if(b_x_mean < r_x_mean){
			flag--;
		}
		else ;

	}
	//std::cout<<"red:"<<r_bottom_cen.size()<<"blue:"<<b_bottom_cen.size()<<std::endl;
		#ifdef debug
		std::cout << "flag:" << int(flag/5.0f)<<std::endl;
	#endif
	//4、如果同时识别红蓝锥桶，根据成对的红蓝锥桶，解算转角t
	if(r_bottom_cen.size()!=0 && b_bottom_cen.size()!=0){//如果同时识别到红蓝锥桶

		if(r_bottom_cen.size() <= b_bottom_cen.size()){ //如果红色锥桶识别的数量比蓝色锥桶少
			for(int i = 0; i < r_bottom_cen.size(); i++){
				float t = cen_angle_solve(redBlocks[i], blueBlocks[i], r_bottom_cen[i], b_bottom_cen[i]);//根据成对的红蓝锥桶，解算转角t
				center_angles.push_back(t);
			}
		}

		else{
			for(int i = 0; i < b_bottom_cen.size(); i++){
				float t = cen_angle_solve(redBlocks[i], blueBlocks[i], r_bottom_cen[i], b_bottom_cen[i]);
				center_angles.push_back(t);
			}
		}
    //以上两个条件是为了，让锥桶可以一一匹配，多余的就不要了

		result = center_angles[0];//无论有几对红蓝锥桶我都只要第一个

		if(flag>-5&&flag<5){//如果flag没+到5就不进行旋转
			result=0.0f;
		}
		save = 0.0f;

	}
//5、红蓝都没识别到则保持直行，
	else if(r_bottom_cen.size() == 0 && b_bottom_cen.size() == 0){
		result = save;//如果都没识别到那么，角度就等于当前角度，保持直行
	}
//6、只有红与只有蓝的情况如下
	else if(r_bottom_cen.size() == 0){
		if(flag == 5){//目前行走判定正常，沿着跑到逆时针旋转，当前方都为蓝色锥桶时逆时针偏转
			result = save;
			save += -acc_angle;
		}
		else if(flag > -5 && flag < 5){ //未判定是正向开车还是倒着开车，默认直行
			result = 0.0f;
		}
		else{//倒行则顺时针
			result = save;
			save += acc_angle;
		}
	}

	else if(b_bottom_cen.size() == 0){
		if(flag == 5){
			result = save;
			save += acc_angle;
		}
		else if(flag > -5 && flag < 5){
			result = 0.0f;
		}
		else{
			result = save;
			save += -acc_angle;
		}
	}

	else{//正常情况不会运行该程序块
		save = 0.0f;
		result = save;
	}
	//给旋转角度添加最大最小值
	if(save > 160.0f)
		save = 160.0f;
	if(save < -160.0f)
		save = -160.0f;

//std::cout<<"result="<<result<<std::endl;
	return result;
}


