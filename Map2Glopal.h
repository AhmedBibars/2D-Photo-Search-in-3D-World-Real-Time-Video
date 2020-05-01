/*function converts 2D pixel-location in the image to 3D glopal-coordinates location*/
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2\opencv.hpp"

#include <opencv2\legacy\legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2\nonfree\gpu.hpp>




cv::Point3f Map2Glopal(cv::Point2f MapPixelLocation,cv::Mat Map,const float MapSizemm_X,const float MapSizemm_Y)
{
	float MapWidthPixel = Map.size().width;
	float MapHeightPixel = Map.size().height;
	cv::Point3f GlopalPostion;
	GlopalPostion.x=MapPixelLocation.x*(MapSizemm_X/MapWidthPixel);             // +-->x
	GlopalPostion.y=MapPixelLocation.y*(MapSizemm_Y/MapHeightPixel);            // |
	GlopalPostion.z=0.0;                                                        // Y      
	return GlopalPostion;
}

