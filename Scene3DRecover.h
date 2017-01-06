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
	BundlerFileLoader sfmLoader;
	vector<SIFTFileLoader> allSiftsCam1;
	vector<SIFTFileLoader> allSiftsCam2;

	vector<vector<DMatch>> siftMatches;
	vector<pair<int, int>> matchedFramesID;
	vector<vector<scenePointOnPair>> allForeGroundScenePoints;//ordered in matched frame, each match for one entry
	vector<vector<scenePoint>> allBackGroundPointsCam1;
	vector<vector<scenePoint>> allBackGroundPointsCam2;

	string sourceFolder1, targetFolder1, maskFolder1;
	string sourceFolder2, targetFolder2, maskFolder2;
	vector<string> cam1ImgNames, cam2ImgNames, cam1SIFTNames, cam2SIFTNames;
	vector<string> cam1MaskNames, cam2MaskNames;

	int FrameW;
	int FrameH;
	map<int, pair<int,int> > cam2Frame;
	map<pair<int,int>, int> frame2Cam;
	int cam1Num;
	int cam2Num;

public:
	void getForeGround3DAllFrames();
	void getBackGround3DAllFrames();
	void getFilePaths(string&, string&);
	void recoverFore3D1F(vector<scenePointOnPair>& out_ScnPoints, string& img1Name, string& img2Name, CameraModel& cam1, CameraModel& cam2, SIFTFileLoader& sfl1, SIFTFileLoader& sfl2);
	void getMatchesSIFTLoader(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch);
	Scene3DRecover();
	~Scene3DRecover();

	CameraModel& getCamModelByFrameId(int camId, int frameId);
	void mapCam2Frame();
};

