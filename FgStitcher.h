#pragma once

#include <stdlib.h>
#include <string>
#include "traj.h"
#include "SFMLib.h"
#include "BundlerFileLoader.h"
#include "SIFTFileLoader.h"
#include "CameraHelper.h"
#include "StitchSolver.h"
#include <vector>
#include "graph.h"
using namespace std;
typedef Graph<int, int, int> GraphType;

class FgStitcher
{
public:
	string warpedFolderL, warpedFolderR;
	string warpedFolderBG;
	GraphType *myGraph;
	vector<cv::Mat> fgLeftSeq, fgRightSeq;
	vector<cv::Mat> bgLeftSeq, bgRightSeq;
	vector<cv::Mat> fgMskLeftSeq, fgMskRightSeq;
	vector<cv::Mat> bgMskLeftSeq, bgMskRightSeq;
	vector<cv::Mat> refine_fgMskLeftSeq, refine_fgMskRightSeq;
	vector<cv::Mat> nodeIDSeq1F;   // Non-grids graph, store node ID at every position
	vector<cv::Mat> nodeTypeSeq1U;
	// default arguments
	float bha_slope = 0.5f;
	int numBinsPerChannel = 16;
	float EDGE_STRENGTH_WEIGHT = 0.95f;
	void loadImageSeqs();


	const float INT32_CONST = 1000;
	const float HARD_CONSTRAINT_CONST = 1000;


#define NEIGHBORHOOD_4_TYPE 1;

	const int NEIGHBORHOOD = NEIGHBORHOOD_4_TYPE;

public:
	void testAllFrame();
	void testTwoFrame();
	FgStitcher();
	~FgStitcher();
};

