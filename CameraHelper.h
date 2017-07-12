#pragma once
#include "SIFTFileLoader.h"
#include "BundlerFileLoader.h"
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/nonfree/nonfree.hpp>  
#include <opencv2/nonfree/features2d.hpp>  
using namespace cv;
using namespace std;
namespace CameraHelper
{
	void changeCoordinate(cv::Point2f& pos, cv::Point2f& center);
	void getSIFTforeground(Mat& img1, Mat& img2, Mat& mask1, Mat& mask2);
	void test(cv::Mat& rotMat, cv::Mat& translateMat, double f);
	void testTri(BundlerFileLoader& bfl);
	cv::Point3f triangulation(CameraModel& cam1, CameraModel& cam2, cv::Point2f& p1, cv::Point2f& p2);
	cv::Point2f get2DViewPosition(CameraModel& cam, cv::Point3f& Pos3D);
	Mat_<double> LinearLSTriangulation(
		Point3d u,//homogenous image point (u,v,1)  
		Matx34d P,//camera 1 matrix  
		Point3d u1,//homogenous image point in 2nd camera  
		Matx34d P1//camera 2 matrix  
		);
};