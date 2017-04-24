// SFMLibTest.cpp : Defines the entry point for the console application.
//
#include <stdlib.h>
#include <string>
#include "traj.h"
#include "SFMLib.h"
#include "BundlerFileLoader.h"
#include "SIFTFileLoader.h"
#include "CameraHelper.h"
#include "StitchSolver.h"

int main()
{	
	StitchSolver solver;
	//solver.prepareForBundler();
	solver.loadReconstruction();
	//solver.preprocessMask();

	//solver.warpFgHomo();
	solver.warpOnMesh();
	//solver.warpMLS();

	/*
	string oriFolder = ("I:\\2016fallproject\\data\\video021_022");
	string maskFolder = ("I:\\2016fallproject\\data\\video021_022_mask");
	string siftMaskResFolder = ("I:\\2016fallproject\\data\\video021_022_recon\\");
	// Handle mask before recovering
	//SIFTHandle::updateSIFTfolder(oriFolder, maskFolder, siftMaskResFolder);


	//BundlerFileLoader bfl(string("I:\\2016fallproject\\data\\video019_020\\1920.nvm.cmvs\\00\\cameras_v2.txt"), string("I:\\2016fallproject\\data\\video019_020\\1920.nvm.cmvs\\00\\bundle.rd.out"));

	Scene3DRecover foreRecover;
	foreRecover.sfmLoader.init(string("I:\\2016fallproject\\data\\video021_022\\2122.nvm.cmvs\\00\\cameras_v2.txt"), string("I:\\2016fallproject\\data\\video021_022\\2122.nvm.cmvs\\00\\bundle.rd.out"));
	foreRecover.getFilePaths(maskFolder, oriFolder);
	foreRecover.mapCam2Frame();
	
	for(int i = 1; i < 30; i++)
		foreRecover.matchedFramesID.push_back(pair<int, int>(i, i));
	foreRecover.getForeGround3DAllFrames();
	foreRecover.getBackGround3DAllFrames();
	


	vector<scenePointOnPair>& ScnPoints = foreRecover.allForeGroundScenePoints[0];
	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;
	for(unsigned i = 0; i < ScnPoints.size(); i++)
	{
		oriPoints.push_back(ScnPoints[i].pos2D_1);
		tgtPoints.push_back(ScnPoints[i].pos2D_2);
		scnPoints.push_back(ScnPoints[i].scenePos);
	}
	vector<scenePoint> BGScnPointsCam1 = foreRecover.allBackGroundPointsCam1[1];
	vector<scenePoint> BGScnPointsCam2 = foreRecover.allBackGroundPointsCam2[1];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
	for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
	{
		oriPoints.push_back(BGScnPointsCam1[i].pos2D);
		scnPoints.push_back(BGScnPointsCam1[i].scenePos);
	}
	for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
		tgtPoints.push_back(BGScnPointsCam2[i].pos2D);


	ViewGenerator vg(oriPoints, tgtPoints, scnPoints, 1280, 720);
	vg.getNewFeaturesPos(foreRecover.getCamModelByFrameId(2, foreRecover.matchedFramesID[0].second));
	vg.getNewMesh();

	Warper warper(Mesh(vg.FrameW, vg.FrameH, vg.GridX, vg.GridY));
	Mat out(vg.FrameH, vg.FrameW, CV_8UC3, Scalar(255, 255, 255));
	warper.warpBilateralInterpolate(imread(foreRecover.cam1ImgNames[1]), vg.deformedMesh, out);
	imshow("warp", out);
	waitKey(0);


	CameraHelper::testTri(bfl);
	CameraHelper::triangulation(bfl.allCameras[10], bfl.allCameras[1], cv::Point2f(-293.04510498, 174.450256348), cv::Point2f(-310.231445313, 129.966949463));
	CameraHelper::test(bfl.allCameras[35].rotation, bfl.allCameras[35].translation, bfl.allCameras[35].focallength);
	CameraHelper::test(bfl.allCameras[34].rotation, bfl.allCameras[34].translation, bfl.allCameras[34].focallength);
	*/
}

//This is an example of usage of SFMlib, please see prototype in SFM.lib
int mainOri()
{	

	//the intrinsic matrix K
	double intriK[3][3]={ 
	{2078.595725, 0.0, 963.103805},
	{0.0,  2103.087505, 760.702205},
	{0.0, 0.0, 1.0}
	};//

	trajgrp* ptrajgrp;
	//trajgrp* atrajgrp;
	//trajgrp* qTrajGrp;
	trajgrp* rTrajGrp=new trajgrp;

	int SF,EF,mLen;
	bool ShowMode;
	char* fileformat;


	mLen=3;
	ShowMode=false;

	//important initial para
	//fileformat="nt%04d.jpg";
	fileformat="CH%02d.jpg";
	//fileformat="DSCF3278%04d.jpg";
	SF=0;
	EF=10;


	printf("Begin SIFT feature selection......\n");
	ptrajgrp=GetFeaturesBySIFT(fileformat,SF,EF,mLen,ShowMode);
	//ptrajgrp->outTrajList("Matlab_Sift_1015_book.m",mLen);

	ptrajgrp->outTrajCpp("Cpp_sift_1111_book.dat",mLen);
	ptrajgrp->outTrajList("mat_sift_1111.m");
	ptrajgrp->GetFeatureColor();
	printf("End SIFT feature selection\n......");

//	ptrajgrp=new trajgrp;
//	ptrajgrp->readTrajCpp("Cpp_sift_1016_book.dat");	
//	ptrajgrp->GetFeatureColor();



	//rTrajGrp->readTrajCpp("Cpp_sift_1016_book.dat");
	//rTrajGrp->outTrajList("Church.m");

//	rTrajGrp->GetFeatureColor();
	//HyperFactorSFM(rTrajGrp,intriK,"CombinedResult.sfm",1000,3);
	TriSBA(ptrajgrp, "FinalResult.sfm", intriK, 5);


//	TriSBA(rTrajGrp,"FinalResult.sfm",intriK,5);
/*	
	printf("Begin KLT feature selection\n");	
	atrajgrp=GetFeaturesByKLT(fileformat,SF,EF,100,80,false);
	printf("End KLT feature selection\n");
	atrajgrp->outTrajList("KLTout_922.m",0);
	atrajgrp->outTrajCpp("CPP_KLTout_1008");

	atrajgrp->DrawFeaturesIndex();

	qTrajGrp=GetScreenFeatureByFrame(ptrajgrp, 0, 3);
	
	qTrajGrp->outTrajCpp("qTrajgrp.dat");

	HyperFactorSFM(rTrajGrp,intriK,"CombinedResult.sfm",1000,3);
*/
	ptrajgrp->TrajRelease();
	rTrajGrp->TrajRelease();
	//qTrajGrp->TrajRelease(); 
	//atrajgrp->TrajRelease();

	//rTrajGrp->TrajRelease();

	system("pause");
	return 0;
}

