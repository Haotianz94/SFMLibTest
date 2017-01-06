#ifndef _STITCHSOLVER_H_
#define _STITCHSOLVER_H_

#include "Scene3DRecover.h"
#include "ViewGenerator.h"
#include "Warper.h"


class StitchSolver
{
public:
	StitchSolver();
	void prepareForBundler();
	void loadReconstruction();
	void testWarping(int);
	void test();

private:
	Scene3DRecover recover;
	ViewGenerator generator;
	Warper warper;

	std::string oriFolder;
	std::string maskFolder;
	std::string siftMaskResFolder;
	std::string cameraList;
	std::string bundlerRes;

	int GridX, GridY;
	int FrameW, FrameH;
};

#endif