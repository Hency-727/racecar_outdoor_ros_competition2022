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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace std;

void task(int ,void*)
{
	int a=1;
}
void GetHSV(string videoPath)
{
cv::Mat src;
	int low_H=0;
	int high_H=180;
	int low_S=0;
	int high_S=255;
	int low_V=0;
	int high_V=255;
	cv::Mat frame;
	cv::VideoCapture Video(videoPath);
	Video>>src;
	cv::namedWindow("image",0);
    cv::createTrackbar("Hmin","image",&low_H,180,task);
    cv::createTrackbar("Hmax","image",&high_H,180,task);

    cv::createTrackbar("Smin","image",&low_S,255,task);
    cv::createTrackbar("Smax","image",&high_S,255,task);

    cv::createTrackbar("Vmin","image",&low_V,255,task);
    cv::createTrackbar("Vmax","image",&high_V,255,task);
	while(1)
{
		cv::Mat HSV;
	cv::Mat dst;
	std::vector<cv::Mat> hsvSplit;

	cv::cvtColor(src, HSV, cv::COLOR_BGR2HSV);

	cv::split(HSV, hsvSplit);
	cv::equalizeHist(hsvSplit[2], hsvSplit[2]);
	cv::merge(hsvSplit, HSV);


		cv::inRange(HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), dst);
	
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	//cv::morphologyEx(dst, dst, cv::MORPH_OPEN, element);
	//cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, element);
	cv::imshow("image",dst);
Video>>src;
	if (cv::waitKey(1)&0xFF==27)
{
	break;
}

}
	cv::destroyAllWindows();
Video.release();


}
int main()
{
cout<<"please changing the HSV for "<<endl;
string videoPath="/dev/video0";
	GetHSV(videoPath);
	return 0;
}
