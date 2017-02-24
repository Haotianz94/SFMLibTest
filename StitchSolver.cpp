#include "StitchSolver.h"
#include "CV_Util2.h"

using namespace std;
StitchSolver::StitchSolver(string folder, string No)
{
	string baseFolder = string("I:\\2016fallproject\\");

	oriFolder = baseFolder + folder + string("\\") + No + string("\\origin");
	maskFolder = baseFolder + folder + string("\\") + No + string("\\mask");
	fgFolder = baseFolder + folder + string("\\") + No + string("\\fg");
	bgFolder = baseFolder + folder + string("\\") + No + string("\\bg");
	//siftMaskResFolder = string("I:\\2016fallproject\\data2\\video001_002_recon\\");
	cameraList = baseFolder + folder + string("\\") + No + string("\\all.nvm.cmvs\\00\\cameras_v2.txt");
	bundlerRes = baseFolder + folder + string("\\") + No + string("\\all.nvm.cmvs\\00\\bundle.rd.out");
	GridX = 20;
	GridY = 20;
}

void StitchSolver::prepareForBundler()
{
	// Handle mask before recovering
	SIFTHandle::updateSIFTfolder(oriFolder, maskFolder, fgFolder, bgFolder);

}

void StitchSolver::loadReconstruction()
{
	recover.sfmLoader.init(cameraList, bundlerRes);
	recover.getFilePaths(maskFolder, fgFolder);

	recover.mapCam2Frame();
	FrameH = recover.FrameH;
	FrameW = recover.FrameW;
	
	for(int i = 0; i < 10; i++)
		recover.matchedFramesID.push_back(pair<int, int>(i, i));
	//recover.matchedFramesID.push_back(pair<int, int>(0, 0));
	
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
	/*
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
	*/
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

void StitchSolver::testWarping_middle(int frameMatchId)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;

	CameraModel cam1 = recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(1, frameId1)]];
	CameraModel cam2 = recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]];

	
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
	imwrite("warp.jpg", out);
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

void StitchSolver::warpFgHomo()
{
	fgWarper = ForegroundWarper(FrameW, FrameH);

	for(int i = 0; i < recover.cam1Num; i++)
	{
		REPORT(recover.cam2ImgNames[i]);
		REPORT(recover.cam1MaskNames[i]);

		Mat origin1 = imread(recover.cam1ImgNames[i]);
		Mat origin2 = imread(recover.cam2ImgNames[i]);
		Mat mask1 = imread(recover.cam1MaskNames[i], 0);
		Mat mask2 = imread(recover.cam2MaskNames[i], 0);

		int idInSFM1 = recover.frame2Cam[make_pair(1, i)];
		int idInSFM2 = recover.frame2Cam[make_pair(2, i)];
		CameraModel newCM = recover.sfmLoader.allCameras[idInSFM1].getMedian(recover
			.sfmLoader.allCameras[idInSFM2]);

		fgWarper.addWarpedPair(recover.sfmLoader.allCameras[idInSFM2], recover
			.allForeGroundScenePoints[i], origin1, origin2, mask1, mask2);
	}
}