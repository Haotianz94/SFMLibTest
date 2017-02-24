#pragma once
#include <string>
#include "BundlerFileLoader.h"
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
using namespace std;

struct siftAbstract
{
	//x, y, color, scale, orientation
	cv::Point2f pos;
	float colorStore4b;
	float scale;
	float orientation;
};

class SIFTFileLoader
{
public:
	int siftNum;  
	cv::Mat mask; //points in this mask will be removed
	vector<siftAbstract> allFeatsAbs;
	vector<vector<unsigned char>> allFeatVecData;
	vector<siftAbstract> refinedFeatsAbs;
	vector<vector<unsigned char>> refinedFeatVecData;
	vector<int> refinedOriIDs;
	vector<bool> ifInMask;
	int keepNum;
public:
	SIFTFileLoader();
	void removeSIFTOutofMask(cv::Mat& mask, string& newfileName);
	void removeSIFTinMask(cv::Mat& mask, string& newfileName);
	SIFTFileLoader(string& siftFileName);
	~SIFTFileLoader();
};

namespace SIFTHandle{
	void updateSIFTfolder(string& imgFolder, string& maskFolder, string& tgtFolder, vector<SIFTFileLoader>& out_AllSFL);
	void updateSIFTfolder(string& imgFolder, string& maskFolder, string& fgFolder, string& bgFolder);
}

