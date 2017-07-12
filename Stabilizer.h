#pragma once

#include "Scene3DRecover.h"
#include "ViewGenerator.h"
#include "Warper.h"
#include "ForegroundWarper.h"

class Stabilizer
{
public:
	Stabilizer();
	Stabilizer(std::string, std::string);
	~Stabilizer();
	void prepareForBundler();
	void loadReconstruction();
	void warpFgHomo();
	void warpOneFrame(CameraModel& newCM, vector<scenePointOnPair>& fgScene, vector<int>& select, Mat origin, Mat mask, bool isSequence1);
	void getSmoothPath();

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
	std::string cameraList;
	std::string bundlerRes;

	int GridX, GridY;
	int FrameW, FrameH;
};

