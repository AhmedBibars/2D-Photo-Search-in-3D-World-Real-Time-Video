#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2\opencv.hpp"
 
//#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>


using namespace std;
using namespace cv;


void ReadCamInterinsicXML(char XMLFileName[], cv::Mat  & CameraMatrix,  cv::Mat & DistortionCoefficients )
{
	//char XMLFileName[]="out_camera_data.xml";
	FileStorage fs2(XMLFileName, FileStorage::READ);
	fs2["Camera_Matrix"] >> CameraMatrix;
	fs2["Distortion_Coefficients"] >> DistortionCoefficients;
}
