#include "StitchSolver.h"
#include "CV_Util2.h"

using namespace std;
StitchSolver::StitchSolver()
{
	oriFolder = string("I:\\2016fallproject\\data\\video021_022");
	maskFolder = string("I:\\2016fallproject\\data\\video021_022_mask");
	siftMaskResFolder = string("I:\\2016fallproject\\data\\video021_022_recon\\");
	cameraList = string("I:\\2016fallproject\\data\\video021_022\\2122.nvm.cmvs\\00\\cameras_v2.txt");
	bundlerRes = string("I:\\2016fallproject\\data\\video021_022\\2122.nvm.cmvs\\00\\bundle.rd.out");
	GridX = 20;
	GridY = 20;
}

void StitchSolver::prepareForBundler()
{
	// Handle mask before recovering
	SIFTHandle::updateSIFTfolder(oriFolder, maskFolder, siftMaskResFolder);

}

void StitchSolver::loadReconstruction()
{
	recover.sfmLoader.init(cameraList, bundlerRes);
	recover.getFilePaths(maskFolder, oriFolder);

	recover.mapCam2Frame();
	FrameH = recover.FrameH;
	FrameW = recover.FrameW;
	
	//for(int i = 0; i < 30; i++)
	//	recover.matchedFramesID.push_back(pair<int, int>(i, i));
	recover.matchedFramesID.push_back(pair<int, int>(0, 0));
	recover.getForeGround3DAllFrames();
	recover.getBackGround3DAllFrames();
}

void StitchSolver::testWarping(int frameMatchId)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;

	vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;
	for(int k = 0; k < 20; k++)
	{
		for(unsigned i = 0; i < ScnPoints.size(); i++)
		{
			oriPoints.push_back(ScnPoints[i].pos2D_1);
			tgtPoints.push_back(ScnPoints[i].pos2D_2);
			scnPoints.push_back(ScnPoints[i].scenePos);
		}
	}
	REPORT(oriPoints.size());
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
	for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
	{
		oriPoints.push_back(BGScnPointsCam1[i].pos2D);
		scnPoints.push_back(BGScnPointsCam1[i].scenePos);
	}
	for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
		tgtPoints.push_back(BGScnPointsCam2[i].pos2D);


	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewMesh();

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	CVUtil::visualizeMeshAndFeatures(imread(recover.cam1ImgNames[frameId1]), out, oriPoints, generator.deformedMesh);
	imshow("match", out);
	waitKey(0);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	warper.warpBilateralInterpolate(imread(recover.cam1ImgNames[frameId1]), generator.deformedMesh, out);
	imshow("warp", out);
	waitKey(0);
}

void StitchSolver::test()
{
	Mat img = imread(recover.cam2ImgNames[0]);
	for(auto& pos: recover.allSiftsCam2[0].allFeatsAbs)
		circle(img, pos.pos, 2, Scalar(255, 0, 0), -1);

	for(auto& pos : recover.allBackGroundPointsCam2[0])
		circle(img, pos.pos2D, 2, Scalar(0, 0, 255), -1);

	imshow("img2", img);
	waitKey(0);
}