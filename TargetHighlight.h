#include <opencv2\legacy\legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2\nonfree\gpu.hpp>


#include "RobustMatcherGPU.h"



#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
using namespace cv;

cv::Mat TargetHighLight(cv::Mat CamFrame,cv::Point3f TargetGlopalPosition,cv::Mat rvec,cv::Mat tvec,cv::Mat cameraIntrinsics,cv::Mat CamDistortion)
{
	std::vector<cv::Point3f> Targets3DVector;       
	std::vector<cv::Point2f> Targets2DVector;
	Targets3DVector.push_back(TargetGlopalPosition);
	cv::projectPoints(Targets3DVector,rvec,tvec,cameraIntrinsics, CamDistortion,Targets2DVector);
	cv::circle(CamFrame, Point(Targets2DVector[0].x, Targets2DVector[0].y), 5, Scalar(0), 2, 8, 0);
	cv::line(CamFrame,Point(CamFrame.cols/2,CamFrame.rows/2),Point(Targets2DVector[0].x, Targets2DVector[0].y),Scalar(0));
	return CamFrame;
}