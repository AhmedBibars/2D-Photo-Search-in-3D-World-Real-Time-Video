/* This code Takes a specific image file, and try to find its real printed-copy in real-time video recived from a camera moves in 3D world.
Regardless any changes in scale and oriatation of the printed-image.
Also you can determin specific Target-point (2D pixel location) in the target Image, and the class will ougment the real-time video with a mark over this location inside the printed-photo*/
*/
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2\opencv.hpp"

#include <opencv2\legacy\legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2\nonfree\gpu.hpp>

#include <opencv2\viz\viz3d.hpp>
#include <opencv2\viz\vizcore.hpp>

#include "RobustMatcherGPU.h"
//#include "ReadCamInterinsicXML.h"
#include "OutputResults.h"
#include "MapCamLocalization.h"
//#include "MapPrepar.h"
#include "TargetHighlight.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, const char** argv)
{
	Mat cameraIntrinsics, CamDistortion;
	ReadCamInterinsicXML("out_camera_data.xml",cameraIntrinsics, CamDistortion);     // load camera parameters
	MapCamLocalization Localizer;
	Localizer.SetCamParameters(cameraIntrinsics,CamDistortion);
	Localizer.SetMapSURFParameters(10000.0,3);   //900.0 // 10000.0,11 or 3
	Localizer.SetCamSURFParameters(900.0,3);       //900.0 // 400.0,5 0r 3
	Localizer.SetMap("Map2.jpg");
	cv::Point3f Target3D(144.5,101.5,0.0);

	cv::Mat MatchImage;
	cvNamedWindow("MatchResult",cv::WINDOW_NORMAL);// CV_WINDOW_AUTOSIZE);
	cv::resizeWindow("MatchResult", 5000, 7000);
	cvNamedWindow("Target", CV_WINDOW_AUTOSIZE);
	cv::viz::Viz3d myWindow("Coordinate Frame");           //Window for viz
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());  /// Add coordinate axes

	CvCapture* capture = 0;
	capture = cvCaptureFromCAM(CV_CAP_ANY); //0=default, -1=any camera, 1..99=your camera
	if (!capture)
		cout << "No camera detected" << endl;	
	if (capture)
	{
		cout << "In capture ..." << endl;
		IplImage* iplImg;
		iplImg = cvQueryFrame(capture);
        cv::Mat frame = iplImg;
		Localizer.SetFrameSize(frame);
		cv::Mat TargetFrame;
		
		for (;;)
		{
			iplImg = cvQueryFrame(capture);
			frame = iplImg;
			if (frame.empty())
				break;
			MatchImage=Localizer.NewFrame(frame);
			imshow("MatchResult",MatchImage);
			if(Localizer.IsLock())
			{ 
				TargetFrame=Localizer.TargetHighLight(frame,Target3D);
				imshow("Traget",TargetFrame);
				cv::viz::WTrajectory VizCamPoseTraj=Localizer.Visualize3D(30);
				myWindow.showWidget("trajectory2",VizCamPoseTraj);
				myWindow.spinOnce(1, true); 
			}
			
			if (waitKey(10) >= 0)
				break;
		}
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("result");
	return 0;
}