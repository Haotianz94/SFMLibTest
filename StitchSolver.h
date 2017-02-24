#ifndef _STITCHSOLVER_H_
#define _STITCHSOLVER_H_

#include "Scene3DRecover.h"
#include "ViewGenerator.h"
#include "Warper.h"
#include "ForegroundWarper.h"


class StitchSolver
{
public:
	StitchSolver(std::string, std::string);
	void prepareForBundler();
	void loadReconstruction();
	void warpFgHomo();

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
	std::string siftMaskResFolder;
	std::string fgFolder;
	std::string bgFolder;
	std::string cameraList;
	std::string bundlerRes;

	int GridX, GridY;
	int FrameW, FrameH;
};

#endif