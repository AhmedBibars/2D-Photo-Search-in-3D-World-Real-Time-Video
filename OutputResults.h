#include <iostream>
#include <fstream>
using namespace std;

void OutputResult (std::vector<cv::Point2f> keypoints3, std::vector<cv::Point3f> Points3Dvector)
{
	ofstream txtfile;
    txtfile.open ("Results.txt");
    txtfile << "Keypoints.\n";
	txtfile << "Keypoints Size =";
	txtfile << keypoints3.size();
	txtfile << "\n";
	for(int i=0;i<keypoints3.size();i++)
	{
		txtfile <<i;
		txtfile <<"   x=";
		txtfile << keypoints3[i].x;
		txtfile <<"   y=";
		txtfile << keypoints3[i].y;
		txtfile << "\n";
	}
	//////////////////////////////////////////////////////

	txtfile << "3D Points vector.\n";
	txtfile << "3D Points vector Size =";
	txtfile << Points3Dvector.size();
	txtfile << "\n";
	for(int i=0;i<Points3Dvector.size();i++)
	{
		txtfile <<i;
		txtfile <<"   x=";
		txtfile << Points3Dvector[i].x;
		txtfile <<"   y=";
		txtfile << Points3Dvector[i].y;
		txtfile <<"   Z=";
		txtfile << Points3Dvector[i].z;
		txtfile << "\n";
	}


    txtfile.close();
}
