#ifndef _FOREGROUND_H_
#define _FOREGROUND_H_
#include <vector>
#include <opencv2\opencv.hpp>
#include "BundlerFileLoader.h"
#include "Scene3DRecover.h"

class ForegroundWarper
{
public:
	std::vector<cv::Mat> warpedFgCam1;
	std::vector<cv::Mat> warpedFgCam2;
	int FrameW; int FrameH;

public:
	ForegroundWarper() {}
	ForegroundWarper(int W, int H): FrameW(W), FrameH(H){}
	void addWarpedPair(CameraModel&, std::vector<scenePointOnPair>&, cv::Mat, cv::Mat, cv::Mat, cv::Mat);
	
private:	
	cv::Mat warpFgHomo(CameraModel&, std::vector<scenePointOnPair>&, std::vector<int>&, cv::Mat, cv::Mat, bool isSequence1);
	Point2f transform(cv::Mat &H, cv::Point2f srcp);
	cv::Vec3b biliner_interpolate(cv::Point2f pos, cv::Mat &img);
};


#endif