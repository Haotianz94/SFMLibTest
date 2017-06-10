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

	//solver.loadReconstruction();

	//solver.warpFGOnMesh(true);
	//solver.warpFGOnMesh(false);

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

