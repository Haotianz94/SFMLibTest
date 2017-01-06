#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace std;
using namespace cv;

#define REPORT(X) cout << #X << ": " << X << "\n"
#define PRINT(X) cout << X << "\n"

struct CameraModel
{
public:
	cv::Mat rotation; //3*3
	cv::Mat translation; //1*3
	double focallength; 
	double distortX;
	double distortY;
};

struct FeatureOneView
{
	int cameraIndex;
	int siftIndex;
	cv::Point2f position2D;
};

struct OneFeatureInWholeScene
{
	cv::Scalar sceneColor;
	cv::Point3f position3D;
	int numOfVisibelCam;
	vector<FeatureOneView> featInAllViews;
};

class BundlerFileLoader
{
public:
	int viewNum;
	int featNum;
	vector<string> viewImgFileName;
	vector<OneFeatureInWholeScene> allFeats;
	vector<CameraModel> allCameras;
	vector<cv::Mat> allImgs;
public:
	BundlerFileLoader();
	int getCameraByImgName(string& imgName);
	BundlerFileLoader(string& cameraFileName, string& bundlerFileName);
	void init(string& cameraFileName, string& bundlerFileName);
	~BundlerFileLoader();
};

