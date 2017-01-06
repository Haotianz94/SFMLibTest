#ifndef _SFMLIB_H
#define _SFMLIB_H

#include <stdio.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


#define GO_NEXT(A) \
   A = A->next;


/***************************
int GetFeaturesBySIFT(char* pcFNameFmt,char* outFileName, int nStartFrameNo, int nEndFrameNo, int MinLen=4,bool ShowMode=false);
-------
This function gets the features from each frame using SIFT algorithm
the return value is a trajgrp structure;

pcFNameFmt: the format of input sequence of image names. e.g. "test%03d.bmp" indicates the names are "test001.bmp, test002.bmp,test003.bmp" format
nStartFrameNo: starting frame number. 
nEndFrameNo:ending frame number.
MinLen: the minimum length of the features. If the features in the frame could not last longer than MinLen it will not be considered as a feature;
ShowMode: if ShowMode=true, the function will generate output image as "Combined_%04d.bmp" and show the result immediately

USAGE:
GetFeaturesBySIFT("test%04d.bmp","SIFTout.m",0,19,5,true);
***************************/
trajgrp* GetFeaturesBySIFT(char* pcFNameFmt, int nStartFrameNo, int nEndFrameNo, int MinLen=4,bool ShowMode=false);

/***************************
int GetFeaturesByKLT(char* pcFNameFmt, char* outFileName,int nStartFrameNo, int nEndFrameNo,int nFeatures=100,int minFeatures=80,bool ShowMode=false);
-------
This function gets the features from each frame using KLT method
the return value is a trajgrp structure;

pcFNameFmt: the format of input sequence of image names. e.g. "test%03d.bmp" indicates the names are "test001.bmp, test002.bmp,test003.bmp" format
nStartFrameNo: starting frame number. 
nEndFrameNo:ending frame number.
nFeatures: the Maximum features that could be extracted in a single frame
minFeatures: the Minimum features that could be extracted in a single frame
ShowMode: if ShowMode=true, the function will generate the output image as "testKLT%04d.jpg" in the folder
***************************/
trajgrp* GetFeaturesByKLT(char* pcFNameFmt, int nStartFrameNo, int nEndFrameNo,int nFeatures=100,int minFeatures=80,bool ShowMode=false);
/***************************
trajgrp* ScreenFeatureByFrame(trajgrp* inputTrajgrp,int n_SF,int n_EF)
-------
This function selects all the features of the same starting frame and ending frame
May be or not be visible to the user in future
***************************/
trajgrp* GetScreenFeatureByFrame(trajgrp* inputTrajgrp,int n_SF,int n_EF);
/***************************
void FactorSFM(trajgrp* ptrajgrp,char* outFileName);
-------
This function use Factorization method to generate the 3D points
ptrajgrp: the pointer to the trajgrp storing all the feature information;
outFileName: the output filename in .SFM
nIt:iteration time
***************************/
void FactorSFM(trajgrp* ptrajgrp,double intK[3][3],char* outFileName,int nIt=3000);
/***************************
int TriSBA(trajgrp* ptrajgrp,char* outFileName);
-------
this function uses triangulation and bundle adjustment to get 3D points.
ptrajgrp: the pointer to the trajgrp of all the feature points in 2D;
outFileName:the output filename in .SFM
***************************/

int TriSBA(trajgrp* ptrajgrp,char* outFileName,double intriK[3][3],double fThreshold,int nMinTrajLen=3);

/***************************
void HyperFactorSFM(trajgrp* ptrajgrp,char* outFileName);
-------
This function uses Serveral Factorization method to generate the 3D points
ptrajgrp: the pointer to the trajgrp storing all the feature information;
outFileName: the output filename in .SFM
nIt:iteration time
Span:frame num. for one Factorization method.
***************************/
void HyperFactorSFM(trajgrp* ptrajgrp,double intK[3][3],char* outFileName,int nIt=1000,int Span=3);
#endif