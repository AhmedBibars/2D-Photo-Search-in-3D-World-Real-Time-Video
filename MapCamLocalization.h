/* This class Takes a specific image file, and try to find its real printed-copy in real-time video recived from a camera moves in 3D world.
Regardless any changes in scale and oriatation of the printed-image.
Also you can determin specific Target-point (2D pixel location) in the target Image, and the class will ougment the real-time video with a mark over this location inside the printed-photo.
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
#include "Map2Glopal.h"

#include "RobustMatcherGPU.h"
#include "ReadCamInterinsicXML.h"
//#include "OutputResults.h"


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
using namespace cv;

class MapCamLocalization
{
private:
	cv::Mat Map;
	cv::Mat RefMap_gray;
	std::vector<cv::KeyPoint> Mapkeypoints;
	cv::Mat MapDescriptor;
	cv::gpu::GpuMat GPUMapDescreptor;
	cv::gpu::GpuMat MapGPUMask;
	cv::SurfFeatureDetector MapSurfDetector;
	cv::SurfDescriptorExtractor MapSudfExtractor;

	cv::Mat Frame;
	cv::Mat Frame_Gray;
	cv::gpu::GpuMat FrameGPUMask;
	cv::Mat cameraIntrinsics;
	cv::Mat CamDistortion;
	std::vector<cv::KeyPoint> Imagekeypoints;

	RobustMatcherGPU rmatch;
	cv::Mat RVec_G_C, TVec_G_C;
    cv::Mat rotationMatrix, TranslatinVecor;
	std::vector<cv::Affine3d> VizCamPoseList;
	bool Lock;

    

public:
	MapCamLocalization(): Lock(false){}
	void SetCamParameters(const cv::Mat Intrinsics,const cv::Mat Distortion)
	{
		Intrinsics.copyTo(cameraIntrinsics);
		Distortion.copyTo(CamDistortion);
	}

	void SetMapSURFParameters(const double thresould,const int NumOctives)
	{
		cv::SurfFeatureDetector SerfDetector(thresould,NumOctives);
		MapSurfDetector=SerfDetector;
	}

	void SetCamSURFParameters(const double thresould,const int NumOctives)
	{
		cv::gpu::SURF_GPU GPUdetector= cv::gpu::SURF_GPU(thresould,NumOctives);
		rmatch.setFeatureDetector(GPUdetector);
		rmatch.setConfidenceLevel(0.98);
		rmatch.setMinDistanceToEpipolar(1.0);
		rmatch.setRatio(0.65f);
		rmatch.refineFundamental(false);
	}
	
	void SetMap(char MapFile[])
	{
		Map = imread(MapFile);                       //load map from file
		cvtColor(Map, RefMap_gray, CV_BGR2GRAY);     //transform map to grayscale
		MapSurfDetector.detect( RefMap_gray,Mapkeypoints);
	    MapSudfExtractor.compute(RefMap_gray,Mapkeypoints, MapDescriptor );
		GPUMapDescreptor.upload(MapDescriptor);
		cv::Mat tempMask2 = Mat::ones(RefMap_gray.size(),CV_8U);
		MapGPUMask.upload(tempMask2);
	}

	cv::Mat NewFrame(cv::Mat NewFrame)
	{
		cv::Mat MatchImage;
		Frame=NewFrame;
		cvtColor(Frame, Frame_Gray, CV_BGR2GRAY);     //transform map to grayscale
		std::vector<DMatch> matches;
		cv::Mat fundamental= rmatch.match(RefMap_gray,Frame_Gray,matches,Mapkeypoints,Imagekeypoints,MapGPUMask,FrameGPUMask,true,GPUMapDescreptor);
		cv::drawMatches( RefMap_gray, Mapkeypoints, Frame, Imagekeypoints, matches,MatchImage);
		
		std::vector<cv::Point2f> Imagekeypoints3;   
		std::vector<cv::Point3f> Points3Dvector; 
		if(matches.size() > 6)
		{
			for(int i=0; i<matches.size();i++)         //loop to compute the global co-ordinated of each cam featcher(features that we found match for them)
			{
				Imagekeypoints3.push_back(Imagekeypoints[matches[i].trainIdx].pt);
				Points3Dvector.push_back(Map2Glopal(Mapkeypoints[matches[i].queryIdx].pt,RefMap_gray,289.0,203.0));
			}
			OutputResult(Imagekeypoints3,Points3Dvector);
			cv::solvePnP(Points3Dvector, Imagekeypoints3, cameraIntrinsics, CamDistortion,RVec_G_C,TVec_G_C,Lock);  // tvec is the location of global-frame-origin in the camera frame
			cv::Rodrigues(RVec_G_C, rotationMatrix); // R is 3x3  this rotation transform from global to camera frame 
			rotationMatrix = rotationMatrix.t();  // rotation of inverse   // now transform form camera frame to global frame
			TranslatinVecor = -rotationMatrix * TVec_G_C; // translation of inverse
			Lock=true;
		}
		else
			Lock=false;
		return MatchImage;
	}


	void SetFrameSize(cv::Mat Frame)
	{
		cvtColor(Frame, Frame_Gray, CV_BGR2GRAY);
		cv::Mat tempMask2 = Mat::ones( Frame_Gray.size(),CV_8U);
		FrameGPUMask.upload(tempMask2);
	}

	cv::Mat TargetHighLight(cv::Mat CamFrame,cv::Point3f TargetGlopalPosition)
	{
		
		std::vector<cv::Point3f> Targets3DVector;       
		std::vector<cv::Point2f> Targets2DVector;
		Targets3DVector.push_back(TargetGlopalPosition);
		cv::projectPoints(Targets3DVector,RVec_G_C,TVec_G_C,cameraIntrinsics, CamDistortion,Targets2DVector);
		cv::circle(CamFrame, Point(Targets2DVector[0].x, Targets2DVector[0].y), 5, Scalar(0), 2, 8, 0);
		cv::line(CamFrame,Point(CamFrame.cols/2,CamFrame.rows/2),Point(Targets2DVector[0].x, Targets2DVector[0].y),Scalar(0));
		return CamFrame;
	}

	viz::WTrajectory Visualize3D(const int TrajectoryLentgh)
	{
		cv::Affine3d VizCamPose(rotationMatrix,TranslatinVecor); 
		VizCamPoseList.push_back(VizCamPose);
		if(VizCamPoseList.size()>TrajectoryLentgh)
			VizCamPoseList.erase(VizCamPoseList.begin());
		viz::WTrajectory VizCamPoseTraj(VizCamPoseList,3);
		return VizCamPoseTraj;
	}

	bool IsLock()
	{
		return Lock;
	}
};
