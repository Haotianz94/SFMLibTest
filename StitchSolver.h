#ifndef _STITCHSOLVER_H_
#define _STITCHSOLVER_H_

#include "Scene3DRecover.h"
#include "ViewGenerator.h"
#include "Warper.h"
#include "ForegroundWarper.h"


class StitchSolver
{
public:
	StitchSolver();
	void loadReconstruction();
	void warpFgHomo();
	void warpFGOnMesh(int, bool, vector<Mat_<Vec2f>>&);
	void warpFGOnMesh(bool isSequence1);
	void warpBGOnMesh(int, bool, vector<Mat_<Vec2f>>&);
	void warpBGOnMesh();

	void prepareForBundler();
	void preprocessMask();
	void preprocessOrigin();
	void extractFeatureFG();
	
	void warpMLS();
	cv::Mat warpMLS(int, bool);

	void testWarping(int);
	void testWarping_middle(int);
	void test();

private:
	Scene3DRecover recover;
	ViewGenerator generator;
	ForegroundWarper fgWarper;
	Warper warper;

	std::string oriFolder;
	std::string maskFolder;
	std::string outFolder;
	std::string siftMaskResFolder;
	std::string fgFolder;
	std::string bgFolder;
	std::string siftFolder;
	std::string cameraList;
	std::string bundlerRes;

	int GridX, GridY;
	int FrameW, FrameH;

	std::string join_path(const char*); 
	void medianFilter(cv::Mat& image, int filter);
	void filterDeformedMesh(vector<Mat_<Vec2f>>&);
	void fillMissedMesh(bool, vector<Mat_<Vec2f>>&);
	void extractFeatureFG(Mat& img, Mat& mask, string featFile);
};

#endif