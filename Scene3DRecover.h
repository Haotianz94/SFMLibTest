#pragma once
#include "CameraHelper.h"
#include <utility>
#include <map>
using namespace std;

struct scenePointOnPair{
	cv::Point3f scenePos;
	cv::Point2f pos2D_1;
	cv::Point2f pos2D_2;
	string img1Name, img2Name;
};

struct scenePoint{
	cv::Point3f scenePos;
	cv::Point2f pos2D;
	string imgName;
};

class Scene3DRecover
{
public:
	string mainFolder;
	string maskFolder;
	string workspaceFolder;
	int startCam2Num;
	BundlerFileLoader sfmLoader;	//only Bg points 
	vector<SIFTFileLoader> allSiftsCam1; //only Fg points
	vector<SIFTFileLoader> allSiftsCam2;

	vector<vector<DMatch>> siftMatches;
	vector<pair<int, int>> matchedFramesID;
	vector<vector<scenePointOnPair>> allForeGroundScenePoints;//ordered in matched frame, each match for one entry
	vector<vector<scenePoint>> allBackGroundPointsCam1;
	vector<vector<scenePoint>> allBackGroundPointsCam2;
	vector<CameraModel> newCamPath;

	string sourceFolder1, targetFolder1, maskFolder1;
	string sourceFolder2, targetFolder2, maskFolder2;
	vector<string> cam1ImgNames, cam2ImgNames, cam1FeatFGNames, cam2FeatFGNames;
	vector<string> cam1MaskNames, cam2MaskNames;

	int FrameW;
	int FrameH;
	map<int, pair<int,int> > cam2Frame;
	map<pair<int,int>, int> frame2Cam;
	int cam1Num;
	int cam2Num;
	int trackNum; 

public:
	void getForeGround3DAllFrames();
	void getBackGround3DAllFrames();
	void trackForeGround(vector<SIFTFileLoader>&);
	void detectFGFeature(Mat& img, vector<SIFTFileLoader>& allSiftsCam);
	void createNewCamPath();

	void getFilePaths(string&, string&);
	void recoverFore3D1F(vector<scenePointOnPair>& out_ScnPoints, CameraModel& cam1, CameraModel& cam2, int, int);
	void getMatchesSIFTLoader(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch);
	void getMatchesSIFTLoader_Ransac(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch);
	Point2f transform(Mat &H, Point2f srcp);
	Scene3DRecover();
	~Scene3DRecover();

	CameraModel& getCamModelByFrameId(int camId, int frameId);
	void mapCam2Frame();
};

