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
	outFolder = baseFolder + folder + string("\\") + No + string("\\out");
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
	
	for(int i = 0; i < 30; i++)
		recover.matchedFramesID.push_back(pair<int, int>(i, i));
	//recover.matchedFramesID.push_back(pair<int, int>(0, 0));
	
	recover.getForeGround3DAllFrames();
	recover.getBackGround3DAllFrames();
}

void StitchSolver::warpOnMesh()
{
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		Mat warped = warpOnMesh(i, true);
		char path[100];
		sprintf_s(path, "%s\\warped1\\img%d.jpg", outFolder.c_str(), i);
		imwrite(path, warped);

		cout << "WarpOnMesh Frame: " << i << endl;
	}
}

Mat StitchSolver::warpOnMesh(int frameMatchId, bool isSequence1)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];

	Mat origin;
	if(isSequence1)
		origin = imread(recover.cam1ImgNames[frameId1]);
	else
		origin = imread(recover.cam2ImgNames[frameId2]);


	vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;
	
	//fill in foreground points
	for(int k = 0; k < 1; k++)
	{
		for(unsigned i = 0; i < ScnPoints.size(); i++)
		{
			if(isSequence1)
			{
				oriPoints.push_back(ScnPoints[i].pos2D_1);
				tgtPoints.push_back(ScnPoints[i].pos2D_2);
			}
			else
			{
				oriPoints.push_back(ScnPoints[i].pos2D_2);
				tgtPoints.push_back(ScnPoints[i].pos2D_1);
			}
			scnPoints.push_back(ScnPoints[i].scenePos);
		}
	}
	REPORT(oriPoints.size());

	//fill in background points
	/*
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
	for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
	{
		if(isSequence1)
		{
			oriPoints.push_back(BGScnPointsCam1[i].pos2D);
			scnPoints.push_back(BGScnPointsCam1[i].scenePos);
		}
		else
			tgtPoints.push_back(BGScnPointsCam1[i].pos2D);
	}
	for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
	{
		if(!isSequence1)
		{
			oriPoints.push_back(BGScnPointsCam2[i].pos2D);
			scnPoints.push_back(BGScnPointsCam2[i].scenePos);
		}
		else
			tgtPoints.push_back(BGScnPointsCam2[i].pos2D);
	}
	*/

	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	
	//generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[idInSFM1].getMedian(recover.sfmLoader.allCameras[idInSFM2]));
	generator.getNewMesh();

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	CVUtil::visualizeMeshAndFeatures(origin, out, oriPoints, generator.deformedMesh);
	//imshow("match", out);
	//waitKey(0);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	warper.warpBilateralInterpolate(origin, generator.deformedMesh, out);
	//imshow("warp", out);
	//waitKey(0);
	return out;
}

void StitchSolver::testWarping(int frameMatchId)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];
	Mat origin = imread(recover.cam1ImgNames[frameId1]);

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
	//generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[idInSFM1].getMedian(recover.sfmLoader.allCameras[idInSFM2]));
	generator.getNewMesh();

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	CVUtil::visualizeMeshAndFeatures(origin, out, oriPoints, generator.deformedMesh);
	imshow("match", out);
	waitKey(0);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	warper.warpBilateralInterpolate(origin, generator.deformedMesh, out);
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

	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		int id1 = recover.matchedFramesID[i].first;
		int id2 = recover.matchedFramesID[i].second;

		Mat origin1 = imread(recover.cam1ImgNames[id1]);
		Mat origin2 = imread(recover.cam2ImgNames[id2]);
		Mat mask1 = imread(recover.cam1MaskNames[id1], 0);
		Mat mask2 = imread(recover.cam2MaskNames[id2], 0);

		int idInSFM1 = recover.frame2Cam[make_pair(1, id1)];
		int idInSFM2 = recover.frame2Cam[make_pair(2, id2)];
		
		CameraModel newCM = recover.sfmLoader.allCameras[idInSFM1].getMedian(recover
			.sfmLoader.allCameras[idInSFM2]);

		fgWarper.addWarpedPair(newCM, recover
			.allForeGroundScenePoints[i], origin1, origin2, mask1, mask2);

		//write to img
		string newFg1 = outFolder + string("\\newFg1.jpg");
		string newFg2 = outFolder + string("\\newFg2.jpg");
		imwrite(newFg1, fgWarper.warpedFgCam1.back());
		imwrite(newFg2, fgWarper.warpedFgCam2.back());
	}
}