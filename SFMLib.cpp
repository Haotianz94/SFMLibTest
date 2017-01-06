//SIFT Header file

#include <iostream>
#include <fstream>

#include "imgfeatures.h"
#include "kdtree.h"
#include "matchseq.h"
#include "minpq.h"
#include "sift.h"
#include "traj.h"
#include "utils.h"
#include "KLTracker.h"
#include "cv_util.h"
#include "SFMLIB.h"


//factorization lib
#include "Factorization.h"

//Vgg_linear_nonliner
#include "vgg_contreps.h"
#include "vgg_X_from_xP_lin.h"
#include "vgg_X_from_xP_nonlin.h"

//EPnP
#include "epnp.h"




int Hello(char* msg)
{
	printf("%s",msg);
	return 1;
}
trajgrp* GetFeaturesBySIFT(char* pcFNameFmt,int nStartFrameNo, int nEndFrameNo, int MinLen,bool ShowMode)
{
	trajgrp* ptrajgrp = new trajgrp;
	//set the starting and ending frame no.
	ptrajgrp->EFNo=nEndFrameNo;
	ptrajgrp->SFNo=nStartFrameNo;

	IplImage* img1, * img2, * pimgBase, * pimgStored;
	struct feature* feat1, * feat2, * feat, * featb;
	struct feature** nbrs, ** nbrsb;
	struct kd_node* kd_root1;
	struct kd_node* kd_root2;
	double d0, d1;
	int n1, n2, k, i;
	int nFrameCnt;

	char pcFNameSrc[128], pcFNameDst[128];
	int nSrcFrameNo, nDstFrameNo;
	int nStartDrawIdx;
	nSrcFrameNo = nStartFrameNo;
	sprintf(pcFNameSrc, pcFNameFmt, nSrcFrameNo);
	img1 = cvLoadImage( pcFNameSrc, 1 );

	if( ! img1 )
		fatal_error( "unable to load image from %s", pcFNameSrc );

	fprintf( stderr, "Finding features in %s...\n", pcFNameSrc );
	n1 = sift_features( img1, &feat1 );
	kd_root1 = kdtree_build( feat1, n1 );
	set_featureno(nSrcFrameNo, n1, feat1); 

	ptrajgrp->imgW=img1->width;
	ptrajgrp->imgH=img1->height;

	if (ShowMode)
	{
		pimgBase = 	cvCreateImage( cvSize(img1->width*4, img1->height*3), IPL_DEPTH_8U, 3 );
		pimgStored = cvCreateImage(cvSize(img1->width, img1->height*12), IPL_DEPTH_8U, 3 );

		DrawSubImg(pimgBase, img1, 0, 0);
		DrawSubImg(pimgStored, img1, 0, 0);
	}

	for (nFrameCnt = 1; nFrameCnt <= nEndFrameNo - nStartFrameNo; nFrameCnt++) {
		nDstFrameNo = nFrameCnt + nStartFrameNo;
		sprintf(pcFNameDst, pcFNameFmt, nDstFrameNo);
		img2 = cvLoadImage( pcFNameDst, 1 );
		if( ! img2 )
			fatal_error( "unable to load image from %s", pcFNameDst );		
		fprintf( stderr, "Finding features in %s...\n", pcFNameDst );
		n2 = sift_features( img2, &feat2 );
		kd_root2 = kdtree_build( feat2, n2 );
		set_featureno(nDstFrameNo, n2, feat2);

	if(ShowMode==true)
	{
		DrawSubImg(pimgStored, img2, 0, (nFrameCnt%12)*img2->height);
		nStartDrawIdx = (nFrameCnt > 11) ? nFrameCnt - 11: 0;
		for (i = 0; i < 12; i++) {
			if ((i/3)%2 == 0)
				DrawSubImgFromSubImg(pimgBase, pimgStored, img2->width, img2->height, 
					(i/3)*img2->width, (i%3)*img2->height, 
					0, ((i+nStartDrawIdx)%12)*img2->height);
			else
				DrawSubImgFromSubImg(pimgBase, pimgStored, img2->width, img2->height, 
					(i/3)*img2->width, (2-(i%3))*img2->height, 
					0, ((i+nStartDrawIdx)%12)*img2->height);
		}
	}

		for( i = 0; i < n1; i++ ) {
			//--- forward matching
			feat = feat1 + i;
			k = kdtree_bbf_knn( kd_root2, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
			if( k == 2 ) {
				d0 = descr_dist_sq( feat, nbrs[0] );
				d1 = descr_dist_sq( feat, nbrs[1] );
				if( d0 < d1 * NN_SQ_DIST_RATIO_THR ) {
					/* printf("fea[%d, %d] (%.2lf,%.2lf) -> fea[%d, %d] (%.2lf, %.2lf) -> ", 
						feat->m_nImgNo, feat->m_nFeaNo, feat->x, feat->y,
						nbrs[0]->m_nImgNo, nbrs[0]->m_nFeaNo, nbrs[0]->x, nbrs[0]->y); */

					//--- backward matching
					featb = feat2 + nbrs[0]->m_nFeaNo;
					k = kdtree_bbf_knn( kd_root1, featb, 2, &nbrsb, KDTREE_BBF_MAX_NN_CHKS );
					if (k == 2 && nbrsb[0]->m_nFeaNo == i) {
						d0 = descr_dist_sq( featb, nbrsb[0] );
						d1 = descr_dist_sq( featb, nbrsb[1] );
						if( d0 < d1 * NN_SQ_DIST_RATIO_THR ) {
							/*
							printf("fea[%d, %d] (%.2lf,%.2lf) -> fea[%d, %d] (%.2lf, %.2lf)\n", 
								featb->m_nImgNo, featb->m_nFeaNo, featb->x, featb->y,
								nbrsb[0]->m_nImgNo, nbrsb[0]->m_nFeaNo, nbrsb[0]->x, nbrsb[0]->y);
								*/
							ptrajgrp->addTrajNode(feat->m_nImgNo, feat->m_nFeaNo, feat->x, feat->y,
								featb->m_nImgNo, featb->m_nFeaNo, featb->x, featb->y);
						} else {
							//printf("not strong enough!\n");
						}
					} else {
						//printf("not matched!\n");
					}
				}
			}
			free( nbrs );
		}

		//--- show and save results in image

		if(ShowMode==true)
		{
			ptrajgrp->drawTrajList(pimgBase, MinLen, 3, img1->width, img1->height, nStartDrawIdx+nStartFrameNo);
			cvNamedWindow( "Combined", 1);
			cvShowImage( "Combined", pimgBase);//this line to change
			sprintf(pcFNameDst, "combined_%04d.jpg", nDstFrameNo);
			cvSaveImage(pcFNameDst, pimgBase);
			cvWaitKey( 1 );
		}

		cvReleaseImage( &img1 );
		kdtree_release( kd_root1 );
		free( feat1 );
		
		img1 = img2;
		kd_root1 = kd_root2;
		feat1 = feat2;
		nSrcFrameNo = nDstFrameNo;
		n1 = n2;
	}
	
	//--- save results in MATLAB .m file
	//ptrajgrp->outTrajList(outFileName, MinLen);
	cvWaitKey( 1000 );

	cvReleaseImage( &img1 );
	kdtree_release( kd_root1 );
	free( feat1 );

	if(ShowMode==true)
	{
		cvReleaseImage( &pimgBase );
		cvReleaseImage( &pimgStored );
	}

	//to confine all the traj has the minimum length
	ptrajgrp->pcFNameFmt=pcFNameFmt;
	ptrajgrp->outTrajCpp("tmp_sift_traj",MinLen);
	//ptrajgrp->Link2Arr();
	ptrajgrp->TrajRelease();

	trajgrp* rTrajGrp=new trajgrp;
	rTrajGrp->readTrajCpp("tmp_sift_traj");
	rTrajGrp->Link2Arr();
	rTrajGrp->GetFeatureColor();
	remove("tmp_sift_traj");

	
	return rTrajGrp;

}


//Function for KLT. Converted by Zhaoyin on Sept.8th 2008
unsigned char* IplImageToChar(IplImage* img)
{
	unsigned char *chImg;
	int w=img->width;
	int h=img->height;
	int c=img->nChannels;

	int i,j,k;
	chImg=new unsigned char[w*h*c];
	CvScalar s;
	for(i=0;i<w;i++)
	{
		for(j=0;j<h;j++)
		{		
			s=cvGet2D(img,j,i); // get the (j,i) pixel value;j is the column, i is the row
			for(k=0;k<c;k++)
			{						
				chImg[i*c+j*(w*c)+k]=(int)s.val[k];
				
			}
		}
	}
	return chImg;
}
bool CheckValid(float *vertex)
{
	if((int)vertex[0]!=-1 && (int)vertex[1]!=-1)
		return true;	
	else
		return false;
}
void WriteFeatureValue(FILE* fpOut,FeatureVector *myFeatureTable, int iStart,int iEnd,int j,int nCnt)
{
	fprintf(fpOut, "pts{%d} = [\n",nCnt);
	int itemp,ytemp;
	for(ytemp=0;ytemp<=1;ytemp++)
	{
		for(itemp=iStart-1;itemp<=iEnd-1;itemp++)
		{						
			fprintf(fpOut," %7.3lf",myFeatureTable[itemp].vertex[j][ytemp]);
		}
		fprintf(fpOut,"\n");
	}

	//add 1 and "];"
	for(itemp=iStart-1;itemp<=iEnd-1;itemp++)
	{						
		fprintf(fpOut," 1.000");
	}
	fprintf(fpOut,"];\n");
}
void SetTrajByFT(traj* trajCur, FeatureVector *myFeatureTable, int iStart,int iEnd, int j,int nCnt)
{
//set the traj link by the feature table;

	int itemp,ytemp;
	double m_x,m_y;

	for(itemp=iStart-1;itemp<=iEnd-1;itemp++)
	{						
		m_x=myFeatureTable[itemp].vertex[j][0];
		m_y=myFeatureTable[itemp].vertex[j][1];
		//addTrajnode(itemp,j,m_x,m_y);
		trajCur->addNode(itemp,j,m_x,m_y);
			
	}		
}
bool IsNewPoint(int intOri,int input)
{
	if( intOri==input)
	{
		return false;
	}
	else
	{
		return true;
	}
}

trajgrp* FeatureTableToTraj(FeatureVector *myFeatureTable,int ImgSize,int nFeatures)
{
//this function convert the feature table (output of KLT) to a trajgrp link structure
//Zhaoyin modified

	//new traj group pointer to return.
	trajgrp* ptrajgrp=new trajgrp;

	traj* trajCur;

	int j;//feature index in myFeatureTable;
	int i;//frame index;
	int nCnt=1;//index of the features;
	bool isEnded;//check if the sequence is continous;
	bool ValidPoint=false;//check if the point is valid
	bool NewPoint=false;//check if the point is different from the previous one
	int intOriValid=-1;//a variable to distinguish different features
	int nTrajNum=0;
	int iStart;

	for(j=0;j<nFeatures;j++)
	{
		isEnded=true;
		
		

		iStart=1;
		intOriValid=-1;
		for(i=iStart-1;i<ImgSize;i++)
		{
			NewPoint=IsNewPoint(intOriValid,myFeatureTable[i].isValid[j]);
			ValidPoint=CheckValid(myFeatureTable[i].vertex[j]);

			intOriValid=myFeatureTable[i].isValid[j];
			if(isEnded==true && ValidPoint==true && NewPoint==true)
			{
				iStart=i+1;
				//Writing the start frame number and initilize a new traj
				if( ptrajgrp->m_ptrajHead==NULL)
				{
					ptrajgrp->m_ptrajHead=new traj;
					trajCur=ptrajgrp->m_ptrajHead;
				}
				else
				{
					trajCur->m_pNext=new traj;
					trajCur=trajCur->m_pNext;
				}
				nTrajNum++;
				trajCur->m_nTrajNo=nTrajNum;
				trajCur->m_nSFNo=i;//it will +1 in outTrajList								
				isEnded=false;
			}
			else if(isEnded==false && ValidPoint==true && NewPoint==false)
			{
				continue;
			}
			else if(isEnded==false && ValidPoint==false)
			{
				//Writing the ending frame number
				trajCur->m_nEFNo=i-1;// the same, will add 1 in outTrajList

				//Set all the trajnode in this traj,//matlab index+1;
				SetTrajByFT(trajCur, myFeatureTable,iStart,i,  j, nCnt);
				isEnded=true;				

				nCnt++;//index increase;
			}
			else if(isEnded==false && ValidPoint==true && NewPoint==true)
			{
				//Writing the ending frame number
				trajCur->m_nEFNo=i-1;// the same, will add 1 in outTrajList

				//Set all the trajnode in this traj,//matlab index+1;
				SetTrajByFT(trajCur, myFeatureTable,iStart,i,  j, nCnt);
				isEnded=true;				

				nCnt++;//index increase;
				//Write end

				//Writing the start frame number and initilize a new traj
				iStart=i+1;
				trajCur->m_pNext=new traj;
				trajCur=trajCur->m_pNext;
				nTrajNum++;
				trajCur->m_nSFNo=i;//it will +1 in outTrajList								
				trajCur->m_nTrajNo=nTrajNum;
				isEnded=false;
				//Start a new one;
			}

		}//end for i(the frame index)
		if(isEnded==false)
		{
			//Writing the ending frame number
			trajCur->m_nEFNo=i-1;
			//Set all the trajnode
			SetTrajByFT(trajCur, myFeatureTable,iStart,i,  j, nCnt);
			isEnded=true;
			nCnt++;
		}
	}//end for j(the feature index)
	ptrajgrp->m_ptrajTail=trajCur;
	ptrajgrp->m_nTrajNum=nTrajNum;

	return ptrajgrp;

}

int PrintKLT(char* pcFName, FeatureVector *myFeatureTable,int ImgSize,int nFeatures)//to Print the features in accordance with SIFT
{	
	//Initializing
	FILE* fpOut;
	if ((fpOut = fopen(pcFName, "wt")) == NULL) {
		printf("Unable to write trajlist file %s.\n", pcFName);
		return 0;
	}

	int j;//feature index in myFeatureTable;
	int i;//frame index;
	int nCnt=1;//index of the features;
	bool isEnded;//check if the sequence is continous;
	bool ValidPoint=false;//check if the point is valid

	for(j=0;j<nFeatures;j++)
	{
		isEnded=true;

		int iStart=1;
		for(i=iStart-1;i<ImgSize;i++)
		{
			ValidPoint=CheckValid(myFeatureTable[i].vertex[j]);
			if(isEnded==true && ValidPoint==true)
			{
				iStart=i+1;
				fprintf(fpOut, "pnSF(%d)=%d; ", nCnt,i+1);
				isEnded=false;
			}
			else if(isEnded==false && ValidPoint==true)
			{
				continue;
			}
			else if(isEnded==false && ValidPoint==false)
			{
				fprintf(fpOut, "pnEF(%d)=%d;\n", nCnt,i);//matlab index+1;
				WriteFeatureValue(fpOut,myFeatureTable,iStart,i,j,nCnt);//matlab index+1;
				isEnded=true;				

				nCnt++;//index increase;
			}
		}//end for i(the frame index)
		if(isEnded==false)
		{
			fprintf(fpOut, "pnEF(%d)=%d;\n", nCnt,i);
			WriteFeatureValue(fpOut,myFeatureTable,iStart,i,j,nCnt);
			isEnded=true;
			nCnt++;
		}
	}//end for j(the feature index)
	fclose(fpOut);

	return 1;
}

IplImage* CharToIplImage(unsigned char* chImg,int w, int h, int c)//w=width, h=height, c=nChannel
{
	IplImage* img=cvCreateImage(cvSize(w,h),IPL_DEPTH_8U,c);
	CvScalar s;

	int i,j,k;

	for(i=0;i<w;i++)
	{
		for(j=0;j<h;j++)
		{			
			
			for(k=0;k<c;k++)
			{						
				s.val[k]=(int)chImg[i*c+j*(w*c)+k];			
			}
			cvSet2D(img,j,i,s);
		}
	}
	return img;
}

trajgrp* GetFeaturesByKLT(char* pcFNameFmt, int nStartFrameNo, int nEndFrameNo,int nFeatures,int minFeatures,bool ShowMode)
{

	int i,j,x,y,xx,yy,random_table[256];

	IMGList ImgHead,*pImg,*pImgTail;
	FeatureVector *myFeatureTable = NULL;
	//Address refers to the return of KLT (FeatureTable)
	for (i=0;i<256;i++)
	{
		j = rand()%3;
		switch(j)
		{
			case 0: random_table[i] = rand()%80; break;
			case 1: random_table[i] = rand()%80+80; break;
			case 2: random_table[i] = rand()%80+160; break;
		}
	}
	//Random number for each feature color (painting)

	pImgTail = &ImgHead;

	//load the image by 'format+number' way

	char pcFNameSrc[128];
	int nSrcFrameNo;
	nSrcFrameNo = nStartFrameNo;
	sprintf(pcFNameSrc, pcFNameFmt, nSrcFrameNo);

	int imgSize ,img_W,img_H;

	char **nameTable;//spelling is mistaken in nameTable (nameTalbe as first)

	*pcFNameSrc;//that is featrueFile

	IplImage* img1=0; 
	img1=cvLoadImage(pcFNameSrc,1);
	if(!img1)
		fatal_error("unable to load image from %s", pcFNameSrc );

	img_W=(*img1).width;
	img_H=(*img1).height;


	imgSize=nEndFrameNo-nStartFrameNo+1;//Get Image size

	nameTable = new char*[imgSize];
	for (i=0;i<imgSize;i++) nameTable[i] = new char[256];

	imgSize=0;
	for(i=nStartFrameNo;i<=nEndFrameNo;i++)
	{
		pImg=new IMGList(img_W,img_H,3);//Prepare for Stored RGB array

		sprintf(pcFNameSrc, pcFNameFmt, i);
		img1=cvLoadImage(pcFNameSrc,1);


		(*pImg).Img=IplImageToChar(img1);

		pImgTail->next=pImg;
		GO_NEXT(pImgTail);

		sprintf(nameTable[imgSize],"testKLT%04d.jpg",i);
		imgSize++;
	}

	printf("Starting Tracking\n");
	myFeatureTable = KLTFeatureRetrievalListMin(ImgHead.next,img_W,img_H,imgSize,nFeatures,minFeatures);
	// img_W / img_H : image width and heigh
	// imgSize : how many images You have in List of ImgHead.next
	// 100: Max. no. of Features
	// 80: Min. no. of Features you require.
	// Return as an Array with 100 (Max no.) rows and ImgSize columns.
	printf("End Tracking\n");

	if(ShowMode==1)
	{
		pImg = ImgHead.next;
		IplImage* imgCV;
		for (i=0;i<imgSize;i++)
		{			
			for (j=0;j<myFeatureTable[i].size;j++)
			if (myFeatureTable[i].isValid[j]>=0)
			{
				  x = (int) (myFeatureTable[i].vertex[j][0] + 0.5);
				  y = (int) (myFeatureTable[i].vertex[j][1] + 0.5);
				  for (yy = y - 2 ; yy <= y + 2 ; yy++)
					for (xx = x - 2 ; xx <= x + 2 ; xx++)  
					  if (xx >= 0 && yy >= 0 && xx < img_W && yy < img_H)  
					  {
						pImg->Img[3*(img_W*yy+xx)+2] = random_table[myFeatureTable[i].isValid[j]%252];
						pImg->Img[3*(img_W*yy+xx)+1] = random_table[myFeatureTable[i].isValid[j]%252+1];
						pImg->Img[3*(img_W*yy+xx)+0] = random_table[myFeatureTable[i].isValid[j]%252+2];
					  }				
 
			}
			imgCV=CharToIplImage(pImg->Img,img_W, img_H, 3);//w=width, h=height, c=nChannel

			if(!cvSaveImage(nameTable[i],imgCV)) printf("Could not save: %s\n",nameTable[i]);//save image
			
			printf("%s Saved.\n",nameTable[i]);
			GO_NEXT(pImg);
		}
		cvReleaseImage( &imgCV );
	}
		//Painting color for each feature in each Image.

		
		//PrintKLT(outFileName, myFeatureTable,imgSize,nFeatures);//a function to print out the result.

		trajgrp* ptrajgrp;//return pointer to store all the feature information
		//set the starting and ending frame no.
		
		//Get all the features to ptrajgrp;
		ptrajgrp=FeatureTableToTraj(myFeatureTable,imgSize,nFeatures);
		ptrajgrp->EFNo=nEndFrameNo;
		ptrajgrp->SFNo=nStartFrameNo;
		ptrajgrp->imgW=img_W;
		ptrajgrp->imgH=img_H;
		ptrajgrp->pcFNameFmt=pcFNameFmt;
		ptrajgrp->Link2Arr();
		ptrajgrp->GetFeatureColor();

		//print FeatureTable		

		for (i=0;i<imgSize;i++)
		{
			delete[] nameTable[i];
		}

	cvReleaseImage( &img1 );
	
	delete[] nameTable;	
	delete[] myFeatureTable;


	return ptrajgrp;

}

//Function to screen all the features between these frames
trajgrp* GetScreenFeatureByFrame(trajgrp* inputTrajgrp,int n_SF,int n_EF)
{	
	trajgrp* ptrajgrp=new trajgrp;
	traj *pCur;//a pointer to follow the input Trajgrp
	traj *outCur;// a pointer to produce the output trajgrp

	ptrajgrp->SFNo=n_SF;
	ptrajgrp->EFNo=n_EF;
	ptrajgrp->pcFNameFmt=inputTrajgrp->pcFNameFmt;

	bool IsLinkMode=(inputTrajgrp->pTrajArr==NULL);
	
	int nFeature=0;//the number of features;

	int i=0;

	for(pCur=inputTrajgrp->m_ptrajHead;pCur!=NULL;pCur=pCur->m_pNext)
	{
		if(pCur->m_nSFNo<=n_SF && pCur->m_nEFNo>=n_EF)
		{
			if(ptrajgrp->m_ptrajHead==NULL)
			{
				if(IsLinkMode)
					ptrajgrp->m_ptrajHead=pCur->GetInterLink(n_SF,n_EF);
				else
					ptrajgrp->m_ptrajHead=inputTrajgrp->GetInterTraj(i,n_SF,n_EF,pCur,pCur->m_nTrajNo);

				outCur=ptrajgrp->m_ptrajHead;
			}
			else
			{
				if(IsLinkMode)
					outCur->m_pNext=pCur->GetInterLink(n_SF,n_EF);			
				else
					outCur->m_pNext=inputTrajgrp->GetInterTraj(i,n_SF,n_EF,pCur,pCur->m_nTrajNo);			

				outCur=outCur->m_pNext;//move to the next;
			}
			ptrajgrp->m_ptrajTail=outCur;
			nFeature++;
		}
		i++;
	}

	if(ptrajgrp->m_ptrajHead==NULL)
	{
		fatal_error("No feature has been collected after screening");
	}
	//Set the width and height of the image;
	ptrajgrp->m_nTrajNum=nFeature;
	ptrajgrp->imgW=inputTrajgrp->imgW;
	ptrajgrp->imgH=inputTrajgrp->imgH;
	ptrajgrp->Link2Arr();
	return ptrajgrp;
}
void CopyIntriArray(double Out[3][3],double In[3][3])
{
	int i,j;
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Out[i][j]=In[i][j];
		}
	}
}

//a function use factorization
//ptrajgrp is the input trajgrp pointer that stores all the feature information
//outFileName is the output filename to store the 3D points, ending by .sfm
void FactorSFM(trajgrp* ptrajgrp,double intK[3][3],char* outFileName,int nIt)
{
	int nImg,nFeature,i,m;


	nImg=ptrajgrp->EFNo - ptrajgrp->SFNo +1;
	nFeature=ptrajgrp->m_nTrajNum;

	FACTORIZATION *myFact = new FACTORIZATION(nImg,nFeature); 
	CopyIntriArray(myFact->intK,intK);//Get the intrinsic matrix for the camera

	traj* trajCur;
	trajnode* trajnodeCur;

	//initializing the pointers
	trajCur=ptrajgrp->m_ptrajHead;
	trajnodeCur=trajCur->m_pTNhead;

	for (i=0;i<nFeature;i++)
	{
		for(m=0;m<nImg;m++)
		{
			myFact->data[m][i][0] = trajnodeCur->m_fx;
			myFact->data[m][i][1] = trajnodeCur->m_fy;		
			trajnodeCur=trajnodeCur->m_pNext;	
		}
		
		trajCur=trajCur->m_pNext;
		if(trajCur!=NULL)
		{
			trajnodeCur=trajCur->m_pTNhead;
		}
	}
	myFact->Execute(nIt);
	myFact->SaveToSFM(outFileName);

}
//infact it is converting trajgrp to array

int Traj2Array(double* dblArr,trajgrp* inTraj, int nFrame)
{
	if(dblArr==NULL||inTraj==NULL)
		return 0;
	
	traj* ptraj=inTraj->m_ptrajHead;
	trajnode* ptrajnode;
	int i=0;
	while(ptraj!=NULL)
	{
		if(ptraj->m_nSFNo<=nFrame && ptraj->m_nEFNo>= nFrame)//the traj contains the target frame
		{
			ptrajnode=ptraj->GetNode(nFrame - ptraj->m_nSFNo);
			dblArr[i]=ptrajnode->m_fx;
			i++;
			dblArr[i]=ptrajnode->m_fy;
			i++;
		}
		ptraj=ptraj->m_pNext;
	}
	return 1;
}
void SetmatEPnP(CvMat* matEPnP)
{
	matEPnP->data.db[0]=1915.6508;
	matEPnP->data.db[1]=979.5291;
	matEPnP->data.db[2]=786.7235;
	matEPnP->data.db[3]=823.4478;
	matEPnP->data.db[4]=-892.8301;
	matEPnP->data.db[5]=1923.513;
	matEPnP->data.db[6]=710.35;
	matEPnP->data.db[7]=-1108.2222;
	matEPnP->data.db[8]=0.0643;
	matEPnP->data.db[9]=0.0574;
	matEPnP->data.db[10]=0.9963;
	matEPnP->data.db[11]=-0.7259;
}
void Arr2IntriMat(double arrK[9],CvMat* matK)
{
	int i;
	for(i=0;i<9;i++)
	{
		arrK[i]=matK->data.db[i];
	}
}
//converting the ptraj to vectors of points
CvMat* Traj2Mat(traj* ptraj)
{
	if(ptraj==NULL)
	{
		printf("invalid input for Traj2Mat");
		return NULL;
	}
	int numPts=ptraj->m_nEFNo - ptraj->m_nSFNo+1;

	CvMat* matRT;
	CvMat* matR;

	matR=cvCreateMat(numPts,3,CV_64FC1);
	//matRT=cvCreateMat(3,numPts,CV_64FC1);

	trajnode* pCur=ptraj->m_pTNhead;
	int i=0;
	while(pCur!=NULL)
	{
		matR->data.db[i]=pCur->m_fx;
		i++;
		matR->data.db[i]=pCur->m_fy;
		i++;
		matR->data.db[i]=1;
		i++;
		pCur=pCur->m_pNext;
	}
	matRT=sfmT(matR);
	cvReleaseMat(&matR);

	return matRT;
}

void MatTo3DbyIndex(CvPoint3D64f* m3D,CvMat* pts3D,Link* ptsVisID)
{
	LinkNode* pnode=ptsVisID->head;
	int i;
	for(i=0;i<ptsVisID->num_node;i++)
	{			
		CvMat* tempM=sfmGetCols(pts3D,pnode->value);
		m3D[i].x=tempM->data.db[0];
		m3D[i].y=tempM->data.db[1];
		m3D[i].z=tempM->data.db[2];		
		cvReleaseMat(&tempM);
		pnode=pnode->next;
	}
}
void MatTo2D(CvPoint2D64f* m2D,CvMat* ptsVis2D)
{
	int i;
	for(i=0;i< ptsVis2D->cols;i++)
	{			
		CvMat* tempM=sfmGetCols(ptsVis2D,i);
		m2D[i].x=tempM->data.db[0];
		m2D[i].y=tempM->data.db[1];
		cvReleaseMat(&tempM);
	}
}
/************************************ RANSAC algorithm **********************************/
int ePnP_RANSAC(   const CvPoint3D64f* m3D, const CvPoint2D64f* m2D,
				   double* matK,
                   uchar* mask, double* pfErr, int count, double* matMRT, double* matRT,
                   double threshold, double p,
                   unsigned rng_seed)
{
    int result = 0;

    const int max_random_iters = 1000;
    const int sample_size = 7;
	double* pfErr_Cur = new double [count];
    uchar* curr_mask = 0;
    uchar* temp_mask = 0;

	epnp PnP; 
    PnP.set_internal_parameters(matK[2], matK[5], matK[0], matK[4]);
    PnP.set_maximum_number_of_correspondences(sample_size);


    CV_FUNCNAME( "ePnP_RANSAC" );

    __BEGIN__;

	double matR_est[3][3], vecT_est[3], fVal;
    CvRNG rng = cvRNG(rng_seed);
    int i, j, k, sample_count, max_samples = 500;
    int best_good_count = 0;

    assert( m3D && m2D && matMRT && 0 < p && p < 1 && threshold > 0 );

    CV_CALL( curr_mask = (uchar*)cvAlloc( count ));
    if( !mask )
    {
        CV_CALL( temp_mask = (uchar*)cvAlloc( count ));
        mask = temp_mask;
    }

    for( sample_count = 0; sample_count < max_samples; sample_count++ )
    {
        int idx[sample_size], n;
		CvPoint3D64f ms3D[sample_size];
		CvPoint2D64f ms2D[sample_size];

        // choose random <sample_size> (=7) points
        for( i = 0; i < sample_size; i++ )
        {
            for( k = 0; k < max_random_iters; k++ )
            {
                idx[i] = cvRandInt(&rng) % count;
                for( j = 0; j < i; j++ )
                    if( idx[j] == idx[i] )
                        break;
                if( j == i )
                {
                    ms3D[i] = m3D[idx[i]];
                    ms2D[i] = m2D[idx[i]];
                    break;
                }
            }
            if( k >= max_random_iters )
                break;
        }

        if( i < sample_size )
            continue;

		//--- Perform ePnP estimation 
		PnP.reset_correspondences();
		for (i = 0; i < sample_size; i++)
			PnP.add_correspondence(ms3D[i].x, ms3D[i].y, ms3D[i].z, ms2D[i].x, ms2D[i].y);
		PnP.compute_pose(matR_est, vecT_est);


		//--- Perform Distance Measure
		int good_count = 0;
		double Xc, Yc, inv_Zc, ue, ve, u, v, fdist;
		for (i = 0; i < count; i++) {
			Xc = matR_est[0][0] * m3D[i].x + matR_est[0][1] * m3D[i].y + matR_est[0][2] * m3D[i].z + vecT_est[0];
			Yc = matR_est[1][0] * m3D[i].x + matR_est[1][1] * m3D[i].y + matR_est[1][2] * m3D[i].z + vecT_est[1];
			inv_Zc = 1.0/(matR_est[2][0] * m3D[i].x + matR_est[2][1] * m3D[i].y + matR_est[2][2] * m3D[i].z + vecT_est[2]);
			ue = matK[2] + matK[0] * Xc * inv_Zc;
			ve = matK[5] + matK[4] * Yc * inv_Zc;
			pfErr_Cur[i] = fdist = sqrt( (m2D[i].x - ue) * (m2D[i].x - ue) + (m2D[i].y - ve) * (m2D[i].y - ve) );
			curr_mask[i] = fdist <= threshold ? 1 : 0;
			good_count += curr_mask[i];
		}

        if( good_count > MAX( best_good_count, sample_size-1 ) )
        {
            double ep, lp, lep;
            int new_max_samples;

            // update the current best "goodness" flags
            memcpy( mask, curr_mask, count );
			memcpy( pfErr, pfErr_Cur, count*sizeof(double));
            best_good_count = good_count;

            // try to update (decrease) <max_samples>
            ep = (double)(count - good_count)/count;
            lp = log(1. - p);
            lep = log(1. - pow(1.0-ep, 7.));
            new_max_samples = cvRound(lp/lep);
            max_samples = MIN( new_max_samples, max_samples );
        }
    }
    if( best_good_count < sample_size )
        EXIT;

    result = 1;

	if( best_good_count >= sample_size) {
		PnP.set_maximum_number_of_correspondences(best_good_count);
		PnP.reset_correspondences();
		for (i = 0; i < count; i++) 
			if (mask[i]) PnP.add_correspondence(m3D[i].x, m3D[i].y, m3D[i].z, m2D[i].x, m2D[i].y);
		PnP.compute_pose(matR_est, vecT_est);
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++)
				matRT[i*4+j] = matR_est[i][j];
			matRT[i*4+3] = vecT_est[i];
		}
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 4; j++) {
				fVal = 0;
				for (k = i; k < 3; k++) { // since matK is upper triangular matrix
					fVal += matK[i*3+k] * matRT[k*4+j];
				}
				matMRT[i*4+j] = fVal;
			}
		}
	}

    __END__;

    cvFree( &temp_mask );
    cvFree( &curr_mask );
	delete[] pfErr_Cur;

    return result;
}
int TriSBA(trajgrp* rtrajgrp,char* outFileName,double intriK[3][3],double fThreshold,int nMinTrajLen)
{
	char* camsfname="tmp_cam.txt";
	char* ptsfname="tmp_pts.txt";
	char* camsoutfname="sbaest_cam.txt";
	char* ptsoutfname = "sbaest_pts.txt";
	char* calfname="calib_tmp.txt";
	char* rgbfname="tmp_rgb.txt";
	char* tmptraj="tmp_minlen.txt";
	char* tmptrajcolor="tmp_minlen_color.txt";
	

	//Get the minmum length
	rtrajgrp->outTrajCpp(tmptraj,nMinTrajLen);
	rtrajgrp->outFeatureColor(tmptrajcolor);

	trajgrp* ptrajgrp=new trajgrp;
	ptrajgrp->readTrajCpp(tmptraj);
	ptrajgrp->GetFeatureColor(tmptrajcolor);//

	
	remove(tmptraj);

	remove(tmptrajcolor);

	//Get points in frame 0 and frame1
	trajgrp* tTrajGrp;
	tTrajGrp=GetScreenFeatureByFrame(ptrajgrp,ptrajgrp->SFNo,ptrajgrp->SFNo+1);

	//tTrajGrp->outTrajCpp("tTrajGrp.dat");

	int NumFeature=tTrajGrp->m_nTrajNum;
	int icount=0;

	double* dblPts1=new double[2*NumFeature];
	double* dblPts2=new double[2*NumFeature];

	Traj2Array(dblPts1,tTrajGrp,ptrajgrp->SFNo);
	Traj2Array(dblPts2,tTrajGrp,ptrajgrp->SFNo+1);

	CvMat* pts1;
	CvMat* pts2;
	CvMat* matF;
	CvMat* matE;
	CvMat* matK;
	//CvMat* matTemp;
	
	//initializing all the matrix;
	matF= cvCreateMat(3,3,CV_64FC1);
	matK=cvCreateMat(3,3,CV_64FC1);
		
	SetK(intriK,matK);//Set the intrinsic matrix to the matK;
	FILE* fpK;
	if( (fpK=fopen(calfname,"wt") ) ==NULL)
	{
		printf("Unable to write the file %s.\n", calfname);
		return 0;
	}
	cvMat_print(fpK,"%7.6f\t",matK);
	fclose(fpK);

	pts1= cvCreateMat(NumFeature,2,CV_64FC1);
	pts2= cvCreateMat(NumFeature,2,CV_64FC1);
	

	pts1->data.db=dblPts1;
	pts2->data.db=dblPts2;
	
	//cvFindFundamentalMat(pts1,pts2,matF,CV_FM_8POINT);	
	cvFindFundamentalMat(pts1,pts2,matF,CV_FM_RANSAC);	
	

	matE=sfmMatMul(sfmT(matK),matF,matK);//there is a sign difference  (-) in computing matE;
	matE=sfmMatMul(matE,-1.0);
	//Get the essential matrix;
	
	//cvMat_print(stdout,"%7.8f ",matF);

	int i;
	CvMat* R1[4];
	CvMat* t1[4];

	E2Rt(matE,R1,t1);
	/*
	cvMat_print(stdout,"%7.3f",R1[0]);
*/
	//--- Step 2.5: Verify solutions
	CvMat* Id;
	Id=cvCreateMat(3,4,CV_64FC1);
	cvSetIdentity(Id);
	
	CvMat* matP[5];
	//for(i=0;i<5;i++)
	//	matP[i]=cvCreateMat(3,4,CV_64FC1);

	matP[0] = sfmMatMul(matK,Id);
	
	/*
	cvMat_print(stdout,"%7.3f",R1[0]);
	cvMat_print(stdout,"%7.3f",t1[0]);
	cvMat_print(stdout,"%7.3f",matP[0]);	
*/
	cvReleaseMat(&Id);
	for(i=0;i<=3;i++)
		matP[i+1]=sfmMatMul(matK,sfmAlignMatH(R1[i],t1[i]));

	//initialize indCam
	CvMat** indCam;
	indCam=new CvMat* [2];
	indCam[0]=cvCreateMat(matP[0]->rows,matP[0]->cols,matP[0]->type);
	indCam[1]=cvCreateMat(matP[0]->rows,matP[0]->cols,matP[0]->type);

	cvCopy(matP[0],indCam[0]);

	CvMat* ptsCur_t1;
	CvMat* ptsCur_t1t;
	CvMat* ptsCur_t2;
	CvMat* ptsCur_t2t;

	//put the pts1 and pts2 in the right way to Matlab
	ptsCur_t1t=sfmT(pts1);
	cvReleaseMat(&pts1);

	pts1=cvCreateMat(3,NumFeature,CV_64FC1);
	SetOnes(pts1);
	PutMatV(ptsCur_t1t,pts1,0);

	ptsCur_t2t=sfmT(pts2);
	cvReleaseMat(&pts2);

	pts2=cvCreateMat(3,NumFeature,CV_64FC1);
	SetOnes(pts2);
	PutMatV(ptsCur_t2t,pts2,0);

	cvReleaseMat(&ptsCur_t1t);
	cvReleaseMat(&ptsCur_t2t);

	CvMat* ptsCur;
	CvMat* P;
	int nSol=0;

	//Set Image size
	
	CvMat *imsize = cvCreateMat(2, 2, CV_64FC1);
	int iRowStep = imsize->step / sizeof( double );
	imsize->data.db[0*iRowStep+0] = ptrajgrp->imgW;
	imsize->data.db[0*iRowStep+1] = ptrajgrp->imgW;
	imsize->data.db[1*iRowStep+0] = ptrajgrp->imgH;
	imsize->data.db[1*iRowStep+1] = ptrajgrp->imgH;

	CvMat* pts3D;
	CvMat* tP;
 
	pts3D=cvCreateMat(4,NumFeature,CV_64FC1);//col is defined by i
	int m;
	double tk;

	CvMat* ptMean;
	CvMat* ptProj1;
	CvMat* ptProj2;

	for(m=1;m<=4;m++)
	{
		cvCopy(matP[m],indCam[1]);//this line to change!
		/*
		printf("printting matP%d\n",m);
		cvMat_print(stdout,"%7.3f ",matP[m]);
*/
		for(i=0;i<=NumFeature-1 ;i++)
		{
			ptsCur_t1=sfmGetCols(pts1,i);
			ptsCur_t2=sfmGetCols(pts2,i);

			ptsCur=sfmAlignMatH(ptsCur_t1,ptsCur_t2);					
/*
			printf("printting ptsCur\n");
			cvMat_print(stdout,"%7.6f ",ptsCur);

			printf("printting indCam0\n");
			cvMat_print(stdout,"%7.6f ",indCam[0]);
			printf("printting indCam1\n");
			cvMat_print(stdout,"%7.6f ",indCam[1]);
*/

			P=vgg_X_from_xP_nonlin( ptsCur,indCam,imsize,2);

			cvReleaseMat(&ptsCur);

			tk=1/(P->data.db[3]);
			tP=sfmMatMul(P,tk);

			cvReleaseMat(&P);
			PutMatH(tP,pts3D,i);

			cvReleaseMat(&ptsCur_t1);
			cvReleaseMat(&ptsCur_t2);
			cvReleaseMat(&tP);
		}
		ptMean=mean(pts3D,2);
		
		ptProj1=sfmMatMul(matP[0],ptMean);
		ptProj2=sfmMatMul(matP[m],ptMean);
		
		/*
		printf("matP\n");
		cvMat_print(stdout,"%7.6f ",matP[m]);
		*/
		if(ptProj1->data.db[2]>0 && ptProj2->data.db[2]>0)
		{
			nSol=m;
			break;
		}		
	}

	cvReleaseMat(&imsize);
	cvReleaseMat(&pts3D);

	//printf("printting matPm\n");
	//cvMat_print(stdout,"%7.6f ",matP[nSol]);
	//--- Step 3: Incremental Initialization & Bundle Adjustment
	//Get to example 2

	int nNumTotalCams=ptrajgrp->EFNo;//ending frame
	int nStartCamNo=ptrajgrp->SFNo+2;//current starting frame, with 2 known
	int nCurCamNo;//Current Camera Number

	int intNumCam=ptrajgrp->EFNo - ptrajgrp->SFNo+1;

	int nNumPts=ptrajgrp->m_nTrajNum;
	
	int j;

	//about CamSBA
	//CamSBA should be of the same length with the num of frames

	CvMat** CamSBA=new CvMat* [intNumCam];
	CvMat** ExtriM=new CvMat* [intNumCam];
	CvMat** Cam;
	for(i=0;i<intNumCam;i++)
	{
		CamSBA[i]=cvCreateMat(3,4,CV_64FC1);
	}

	cvCopy(matP[0],CamSBA[0]);
	cvCopy(matP[nSol],CamSBA[1]);

	
	int nEndCamNo;
	int pnSF_i;
	int pnEF_i;
	Link* pnGlobalOutlierNo=new Link;
	traj* trajCur;
	traj* tempTraj;
	
	cvReleaseMatGrp(indCam,2);//clear indCam;

	//begin the whole big loop
	for(nCurCamNo=nStartCamNo; nCurCamNo<=nNumTotalCams; nCurCamNo++)
	{
		printf("Current Frame Number = %d.\n", nCurCamNo);

		trajCur=ptrajgrp->m_ptrajHead;
				
		pts3D=cvCreateMat(4,nNumPts,CV_64FC1);

		int NumAvPts=0;
		CvMat* ptsVis2D=NULL;
		Link* ptsVisID=new Link;

		for(i=0;i<nNumPts;i++)
		{
			//Matlab not translated
			//clear indCam;
			//if (length(find(pnGlobalOutlierNo==i)) > 0), continue; end;
			//continue should go with

			if( pnGlobalOutlierNo->find(i) )
			{
				if(trajCur->m_pNext!=NULL)
					trajCur=trajCur->m_pNext;
				continue;
			}
					
			pnSF_i=trajCur->m_nSFNo;
			pnEF_i=trajCur->m_nEFNo;

			if(pnSF_i>nCurCamNo-nMinTrajLen+1)
				break;
			
			if (pnEF_i > nCurCamNo-1) 			
				nEndCamNo = nCurCamNo-1;			
			else
				nEndCamNo = pnEF_i;
			
			tempTraj=trajCur->GetInterLink(pnSF_i,nEndCamNo);
			//the index should be verified 'nEndCamNo'
			//ptsCur = pts{i}(:, 1:nEndCamNo-pnSF(i)+1);

			ptsCur=Traj2Mat(tempTraj);
			
			tempTraj->Release();
			/*
			//for debug
			printf("printting ptsCur\n");
			cvMat_print(stdout,"%7.6f ",ptsCur);
			*/
			int indCamSize=nEndCamNo-pnSF_i+1;

			indCam=new CvMat*[indCamSize];

			for(m=0;m<indCamSize;m++)
				indCam[m]=cvCreateMat(3,4,CV_64FC1);

			imsize=cvCreateMat(2,indCamSize,CV_64FC1);
			
			for(j=pnSF_i;j<=nEndCamNo;j++)
			{
				cvCopy(CamSBA[j - ptrajgrp->SFNo],indCam[j-pnSF_i]);	
				//the index of CamSBA should be verified. Seems wrong in original code
				// indCam{j-pnSF(i)+1} = CamSBA{j};
				//debug
				/*
				printf("printting indCam %d\n",j-pnSF_i);
				cvMat_print(stdout,"%7.6f ",indCam[j-pnSF_i]);
				*/
				imsize->data.db[j-pnSF_i]=ptrajgrp->imgW;
				imsize->data.db[j-pnSF_i+indCamSize]=ptrajgrp->imgH;
			}			
			/*
			printf("printting imsize %d\n");
			cvMat_print(stdout,"%7.6f ",imsize);
			*/
			P=vgg_X_from_xP_nonlin( ptsCur,indCam,imsize,indCamSize);
						

			tk=1/(P->data.db[3]);

			tP=sfmMatMul(P,tk);

			cvReleaseMat(&P);
			PutMatH(tP,pts3D,i);

			/*				
			//debug
			printf("printting tP\n");
			cvMat_print(stdout,"%7.6f ",tP);
			*/
			/*
			printf("printting pts3D\n");
			cvMat_print(stdout,"%7.3f",pts3D);
			*/

			//Matlab Code
			       
			if (pnEF_i >= nCurCamNo)
			{

				// visible points for this specific camera
				tempTraj=trajCur->GetInterLink(nCurCamNo,nCurCamNo);

				if(ptsVis2D==NULL)//ptsVis2D(:, nVisPtCnt) = pts{i}(:, nCurCamNo-pnSF(i)+1);
				{					
					ptsVis2D=Traj2Mat(tempTraj);					
				}
				else
				{
					CvMat* tempM=Traj2Mat(tempTraj);
					CvMat* tempK=sfmAlignMatH(ptsVis2D,tempM);
					cvReleaseMat(&ptsVis2D);//avoid memory leak

					ptsVis2D=cvCreateMat(tempK->rows,tempK->cols,tempK->type);

					cvCopy(tempK,ptsVis2D);
					cvReleaseMat(&tempM);
					cvReleaseMat(&tempK);
				}
				ptsVisID->AddValue(i);//% index for 3D points
				/*
				printf("printting ptsVis2D\n");
				cvMat_print(stdout,"%7.6f ",ptsVis2D);
				
				*/
				//tempTraj->Release();
			}
					
			if(trajCur->m_pNext!=NULL)
				trajCur=trajCur->m_pNext;

			cvReleaseMatGrp(indCam,indCamSize);
			cvReleaseMat(&tP);
			cvReleaseMat(&ptsCur);
		
		}

		/*
		//for debug
		printf("printting pts3D\n");
		cvMat_print(stdout,"%7.3f",pts3D);
		*/

		//EPnp....waiting for Kevin
		//EPnp....waiting for Kevin
		//EPnp....waiting for Kevin
		//double fThreshold = 5;//a variable might be control out.
		int nNumPts_epnp=ptsVisID->num_node;

		CvPoint3D64f* m3D = new CvPoint3D64f [nNumPts_epnp];
		CvPoint2D64f* m2D = new CvPoint2D64f [nNumPts_epnp];
		uchar* mask = new uchar [nNumPts_epnp];
		double* pfErr = new double [nNumPts_epnp];
		const unsigned rng_seed = 0xffffffff;
		
		double matMRT[12], matRT[12];
		double matKtemp[9];	
		Arr2IntriMat(matKtemp,matK);

		MatTo2D(m2D,ptsVis2D);

		//cvMat_print(stdout,"%7.6f ",ptsVis2D);
		MatTo3DbyIndex(m3D,pts3D,ptsVisID);

		int tempk;
		tempk=ePnP_RANSAC(m3D, m2D, matKtemp, mask, pfErr, nNumPts_epnp, matMRT, matRT, fThreshold, 0.99, rng_seed);
		//error produced by pts3D
		//int nNumOutlier = 0;

		int iEPnP;
		int iValue;
		for (iEPnP = 0; iEPnP < nNumPts_epnp; iEPnP++)
		{	
			if (mask[iEPnP] == 0) 
			{
				iValue=ptsVisID->GetValue(iEPnP);
				pnGlobalOutlierNo->AddValue(iValue);
			}
		}
		CvMat* matRTtemp=cvCreateMat(3,4,CV_64FC1);
		cvInitMatHeader(matRTtemp,3,4,CV_64FC1,matRT);
		cvMat_print(stdout,"%7.6f ",matRTtemp);



		
		CvMat* matEPnP;

		matEPnP=sfmMatMul(matK,matRTtemp);
		
		printf("printting matEPnP\n");
		cvMat_print(stdout,"%7.6f ",matEPnP);

		cvReleaseMat(&matRTtemp);
		
		//--- SBA
		//Camera Parameter file generation
		int sizeCam=nCurCamNo - ptrajgrp->SFNo + 1;
		Cam=new CvMat* [sizeCam];
		for(m=0;m<=sizeCam-2;m++)
		{
			Cam[m]=cvCreateMat(3,4,CV_64FC1);
			cvCopy(CamSBA[m],Cam[m]);
		}
		Cam[sizeCam-1]=cvCreateMat(3,4,CV_64FC1);
		cvCopy(matEPnP,Cam[sizeCam-1]);
		/*
		//for debug
		printf("printting matEPnP\n");
		cvMat_print(stdout,"%7.3f ",matEPnP);
		*/
		FILE* fpOut;
		if ((fpOut = fopen(camsfname, "wt")) == NULL) {
		printf("Unable to write the file %s.\n", camsfname);
		return 0;
		}
		CvMat* q;
		q=cvCreateMat(1,4,CV_64FC1);

		CvMat* t;
		t=cvCreateMat(1,3,CV_64FC1);

		for(i=0;i<=nCurCamNo - ptrajgrp->SFNo;i++)
		{
			/*
			//for debug
			printf("printting Cam[i]\n");
			cvMat_print(stdout,"%7.3f ",Cam[i]);
			*/
			projmat2qt(Cam[i],matK,q,t);

			cvMat_print(fpOut,"%.12f ",q,1);
			cvMat_print(fpOut,"%.12f ",t,1);
			fprintf(fpOut,"\n");
		}
		fclose(fpOut);
		
		FILE* fpOutRGB;
		// Feature Points
		fpOut = fopen(ptsfname, "w");
		fpOutRGB=fopen(rgbfname,"w");
		if (fpOut == NULL) {
		printf("Unable to write the file %s.\n", rgbfname);
		return 0;
		}
		if(fpOutRGB==NULL)
		{
			printf("Unable to write the file %s.\n", rgbfname);
			return 0;
		}
		trajCur=ptrajgrp->m_ptrajHead;//get 2D points
		icount=0;
		int iskip=0;
		for(i=0;i<nNumPts;i++)
		{
			//Matlab not translated
			//clear indCam;
			pnSF_i=trajCur->m_nSFNo;
			pnEF_i=trajCur->m_nEFNo;
			if (pnGlobalOutlierNo->find(i))
			{	
				trajCur=trajCur->m_pNext;
				iskip++;
				continue; 
			}			

			if(pnSF_i>nCurCamNo-nMinTrajLen+1)
				break;
			
			if (pnEF_i > nCurCamNo) 			
				nEndCamNo = nCurCamNo;			
			else
				nEndCamNo = pnEF_i;

			CvMat* pts3D_i;
			CvMat* pts3D_ir;

			pts3D_i=sfmGetCols(pts3D,i,i);
			pts3D_ir=sfmGetRows(pts3D_i,0,2);

			cvMat_print(fpOut,"%.12f ",pts3D_ir,1);
			cvReleaseMat(&pts3D_i);
			cvReleaseMat(&pts3D_ir);

			fprintf(fpOut,"%d",nEndCamNo-pnSF_i+1);//the index remains a problem
			// fprintf(fpOut, '%.12f %.12f %.12f %d', pts3D(1:3, i), nEndCamNo-pnSF(i)+1);
				
			tempTraj=trajCur->GetInterLink(pnSF_i,nEndCamNo);
			trajnode* ptrajnode=tempTraj->m_pTNhead;
			
			//this index issue is very tricky
			//it is revised for index issue
			j=pnSF_i - ptrajgrp->SFNo; //save for 
			
			while(ptrajnode!=NULL)
			{
				fprintf(fpOut," %d %.12f %.12f",j,ptrajnode->m_fx,ptrajnode->m_fy);
				fprintf(fpOutRGB,"%d %d %d\n",(int)(trajCur->RGB[0]),(int)(trajCur->RGB[1]),(int)(trajCur->RGB[2]));
				//differ in matlab
				// fprintf(fpOut, ' %d %.12f %.12f', j-1, pts{i}(1:2,j-pnSF(i)+1));
				j++;
				ptrajnode=ptrajnode->m_pNext;
			}
			fprintf(fpOut,"\n");
			trajCur=trajCur->m_pNext;
			icount++;
		}
		fclose(fpOut);
		fclose(fpOutRGB);
		//printf("icount=%d,iskip=%d,total points=%d\n",icount,iskip,i);

		//cvCopy(matEPnP,CamSBA[nCurCamNo]);//This line is just for test
		cvReleaseMatGrp(Cam,sizeCam);
		cvReleaseMat(&pts3D);
		cvReleaseMat(&imsize);
		cvReleaseMat(&ptsVis2D);
		ptsVisID->Release();

		char mycmd[128];

		sprintf(mycmd,"%s %s %s %s %s %s","eucsbademo.exe",camsfname, ptsfname, calfname, camsoutfname, ptsoutfname);
		printf("%s",mycmd);
		system(mycmd);
		//system("pause");

		//%--- Compare original setting//did not compare

		//read back the data
		CvMat* qtsbaR;
		CvMat* qtR;
		CvMat* qtT;
		CvMat* qtsba;

		qtR=cvCreateMat(3,3,CV_64FC1);
		qtsbaR=cvCreateMat(1,4,CV_64FC1);
		qtT=cvCreateMat(3,1,CV_64FC1);
		qtsba=cvCreateMat(3,4,CV_64FC1);
		
		FILE* fpIn;
		if( (fpIn=fopen(camsoutfname,"r"))==NULL)
		{
			printf("Cannot open file %s.",camsoutfname);
			return 0;
		}
		for(i=0;i<=nCurCamNo - ptrajgrp->SFNo;i++)//tricky index issue
		{
			
			//fscanf(fpIn,"%lf %lf %lf %lf %lf %lf %lf\n",&a,&b,&c,&d,&e,&f,&g);
			fscanf(fpIn,"%lf %lf %lf %lf %lf %lf %lf\n",&(qtsbaR->data.db[0]),&(qtsbaR->data.db[1]),&(qtsbaR->data.db[2]),&(qtsbaR->data.db[3]),
				&(qtT->data.db[0]),&(qtT->data.db[1]),&(qtT->data.db[2]));
			//cvMat_print(stdout,"%7.6f ",qtsbaR);

			qtR=q2rot(qtsbaR);
			//cvMat_print(stdout,"%7.6f ",qtR);
			//cvMat_print(stdout,"%7.6f ",qtT);
			PutMatH(qtR,qtsba,0);
			PutMatH(qtT,qtsba,3);
			ExtriM[i]=cvCreateMat(3,4,CV_64FC1);
			cvCopy(qtsba,ExtriM[i]);
			cvMatMul(matK,qtsba,CamSBA[i]);
/*
			printf("CamSBA %d\n",i);
			cvMat_print(stdout,"%7.6f ",CamSBA[i]);
*/
		}
		cvReleaseMat(&qtsbaR);
		cvReleaseMat(&qtT);
		cvReleaseMat(&qtR);
		cvReleaseMat(&qtsba);
		fclose(fpIn);

	}//end big for loop
	
	FILE *finalout;
	FILE *finalin;
	FILE *finalRGB;
	if( (finalin=fopen(ptsoutfname,"r"))==NULL)
	{
		printf("Cannot open file %s.",ptsoutfname);
		return 0;
	}
	if( (finalRGB=fopen(rgbfname,"r"))==NULL)
	{
		printf("Cannot open file %s.",rgbfname);
		return 0;
	}
	if( (finalout=fopen(outFileName,"wt"))==NULL)
	{
		printf("cannot open %s\n",outFileName);
		return 0;
	}
	fprintf(finalout,"\n");
	fprintf(finalout,"#LABEL 1\n");
	fprintf(finalout,"\n");
	fprintf(finalout,"#POINT %d\n",icount);
	double p3dx,p3dy,p3dz;
	int fr,fg,fb;
	while( (fscanf(finalin,"%lf %lf %lf\n",&p3dx,&p3dy,&p3dz)!=EOF) && (fscanf(finalRGB,"%d %d %d\n",&fr,&fg,&fb)!=EOF) )
	{
		fprintf(finalout,"%7.6lf %7.6lf %7.6lf \t %d %d %d\n",p3dx,p3dy,p3dz,fr,fg,fb);
	}

	fprintf(finalout,"\n\n");
	for(i=0;i<intNumCam;i++)
	{
		fprintf(finalout,"#CAMERA %d\n",i);
		fprintf(finalout,"camera_scale 2.0\n");
		fprintf(finalout,"camera_intrinsic\n");
		cvMat_print(finalout,"%7.6f ",matK,2);
		fprintf(finalout,"camera_extrinsic\n");
		cvMat_print(finalout,"%7.6f ",ExtriM[i]);
		fprintf(finalout,"\n");
	}
		
	fclose(finalout);
	fclose(finalin);
	fclose(finalRGB);


	//delete all the temp files;
	remove(camsfname);
	//char* camsfname="tmp_cam.txt";
	remove(ptsfname);
	//char* ptsfname="tmp_pts.txt";
	remove(camsoutfname);
	//char* camsoutfname="sbaest_cam.txt";
	remove(ptsoutfname);
	//char* ptsoutfname = "sbaest_pts.txt";
	remove(calfname);
	//char* calfname="calib_tmp.txt";
	remove(rgbfname);
	//char* rgbfname="tmp_rgb.txt";

	cvReleaseMatGrp(CamSBA,intNumCam);
	cvReleaseMat(&matK);
	cvReleaseMat(&matF);
	cvReleaseMat(&matE);

	delete dblPts1;
	delete dblPts2;

	cvReleaseMat(&pts1);
	cvReleaseMat(&pts2);
	cvReleaseMatStaGrp(R1,4);
	cvReleaseMatStaGrp(t1,4);

	cvReleaseMatStaGrp(matP,5);

	ptrajgrp->TrajRelease();

	return 1;

}

//A Function : Hyper Factorization
//This Function similar to "FactorSFM", but can handle long period images
void HyperFactorSFM(trajgrp* ptrajgrp,double intK[3][3],char* outFileName,int nIt,int Span)
{
	int nImg,nFeature,iPiece,i,j,k,m,nPiece;
	char buf[64];

	nImg=ptrajgrp->EFNo - ptrajgrp->SFNo +1;
	nPiece =  nImg - Span +1;

	if (nImg<Span) return;
	if (Span<3) return;

	traj* trajCur;
	trajnode* trajnodeCur;
	trajgrp** myTraj;
	double **pTrans = new double*[nPiece-1];
	for (i=0;i<nPiece-1;i++) pTrans[i] = new double[16];

	FACTORIZATION **myFact,*oneFactor;

	myFact = new FACTORIZATION *[nPiece];
	myTraj = new trajgrp *[nPiece];

	for (iPiece=0;iPiece<nPiece;iPiece++)//Calculate for each piece
	{
		myTraj[iPiece]=GetScreenFeatureByFrame( ptrajgrp,ptrajgrp->SFNo+iPiece,ptrajgrp->SFNo+iPiece+Span-1);
		
		printf("doing factorization for piece-%d ...",iPiece);
		nFeature = myTraj[iPiece]->m_nTrajNum;
		oneFactor = new FACTORIZATION(Span,nFeature); //Initialize
		CopyIntriArray(oneFactor->intK,intK);//Get the intrinsic matrix for the camera

		//initializing the pointers
		trajCur=myTraj[iPiece]->m_ptrajHead;
		trajnodeCur=trajCur->m_pTNhead;

		for (i=0;i<nFeature;i++)
		{
			for(m=0;m<Span;m++)
			{
				oneFactor->data[m][i][0] = trajnodeCur->m_fx;
				oneFactor->data[m][i][1] = trajnodeCur->m_fy;		
				trajnodeCur=trajnodeCur->m_pNext;	
			}
			trajCur=trajCur->m_pNext;
			if(trajCur!=NULL) trajnodeCur=trajCur->m_pTNhead;
		}
		oneFactor->Execute(nIt);
		sprintf(buf,"out_deb-%.2d.sfm",iPiece);
		oneFactor->SaveToSFM(buf);
		
		myFact[iPiece] = oneFactor;
		printf("done\n");

	}

	int *tab1=NULL,*tab2=NULL,num;
	double Mapping[16],**p1,**p2,scale1,scale2,pro3D[3],center1[3],center2[3],dist1[3],dist2[3];
	for (i=0;i<nPiece-1;i++)
	{
		myTraj[i]->CompareIndexWithOther(myTraj[i+1],&tab1,&tab2,&num);	
		//Normalize second 3D
		for (k=0;k<3;k++)
		{	center1[k]=0.0;
			center2[k]=0.0;		}
		scale1=0;scale2=0;
		for (j=0;j<num;j++)
			for (k=0;k<3;k++)
			{	center1[k] += myFact[i]->p3D[tab1[j]][k];
				center2[k] += myFact[i+1]->p3D[tab2[j]][k];	}
		for (k=0;k<3;k++)
		{	center1[k]/=float(num);
			center2[k]/=float(num);		}
		for (j=0;j<num;j++)
		{
			for (k=0;k<3;k++)
			{
				dist1[k] = myFact[i]->p3D[tab1[j]][k] - center1[k];
				dist2[k] = myFact[i+1]->p3D[tab2[j]][k] -center2[k];
			}
			scale1 += vector3L(dist1);
			scale2 += vector3L(dist2);
		}
		myFact[i+1]->ApplyScaling(scale1/scale2);

		p1 = new double *[num];
		p2 = new double *[num];
		for (j=0;j<num;j++)
		{ p1[j] = new double[3]; p2[j] = new double[3];	}
		
		for (j=0;j<num;j++)
			for (k=0;k<3;k++)
			{   p1[j][k] = myFact[i]->p3D[tab1[j]][k];
				p2[j][k] = myFact[i+1]->p3D[tab2[j]][k];		}

		Mapping_Matrix(Mapping,p1,p2,num);// p1 = M * p2
		myFact[i+1]->ApplyMapping(Mapping);

		delete[] tab1;  tab1 = NULL;
		delete[] tab2;	tab2 = NULL;
	}

	nFeature =0;
	for (iPiece=0;iPiece<nPiece;iPiece++)//Calculate for each piece
		nFeature += myFact[iPiece]->nFeature;

	FILE *stream = fopen(outFileName,"w");
	fprintf(stream,"This file is generated by multiple Factorization\n\n");
	fprintf(stream,"#LABEL 1\n\n");
	fprintf(stream,"#POINT %d\n",nFeature);
	
	for (iPiece=0;iPiece<nPiece;iPiece++)
	{	int R_color = rand()%200+55;
		int G_color = rand()%200+55;
		int B_color = rand()%200+55;
		for (j=0;j<myFact[iPiece]->nFeature;j++)
			fprintf(stream,"%f %f %f %d %d %d\n",myFact[iPiece]->p3D[j][0],myFact[iPiece]->p3D[j][1],myFact[iPiece]->p3D[j][2],R_color,G_color,B_color);
	}
	fprintf(stream,"\n\n");
	for (iPiece=0;iPiece<nPiece;iPiece++)
	{
		for (j=0;j<myFact[iPiece]->nImg;j++)
		{
			fprintf(stream,"#CAMERA P%.3d-%.1d\n",iPiece,j);
			fprintf(stream,"camera_scale 2.0\ncamera_intrinsic\n");
			fprintf(stream,"%f %f %f\n",myFact[iPiece]->intK[0][0],myFact[iPiece]->intK[0][1],myFact[iPiece]->intK[0][2]);
			fprintf(stream,"%f %f %f\n",myFact[iPiece]->intK[1][0],myFact[iPiece]->intK[1][1],myFact[iPiece]->intK[1][2]);
			fprintf(stream,"%f %f %f\n",myFact[iPiece]->intK[2][0],myFact[iPiece]->intK[2][1],myFact[iPiece]->intK[2][2]);
			fprintf(stream,"camera_extrinsic\n");
			fprintf(stream,"%f %f %f %f\n",myFact[iPiece]->perspMatrix[j].data[0][0],myFact[iPiece]->perspMatrix[j].data[0][1],myFact[iPiece]->perspMatrix[j].data[0][2],myFact[iPiece]->perspMatrix[j].data[0][3]);
			fprintf(stream,"%f %f %f %f\n",myFact[iPiece]->perspMatrix[j].data[1][0],myFact[iPiece]->perspMatrix[j].data[1][1],myFact[iPiece]->perspMatrix[j].data[1][2],myFact[iPiece]->perspMatrix[j].data[1][3]);
			fprintf(stream,"%f %f %f %f\n\n",myFact[iPiece]->perspMatrix[j].data[2][0],myFact[iPiece]->perspMatrix[j].data[2][1],myFact[iPiece]->perspMatrix[j].data[2][2],myFact[iPiece]->perspMatrix[j].data[2][3]);
		}
	}

	fclose(stream);

	for (i=0;i<nPiece;i++)
	{
		myFact[i]->Release();
		myTraj[i]->TrajRelease();
	}
	delete[] myFact;
	delete[] myTraj;

	for (i=0;i<nPiece-1;i++)
		delete[] pTrans[i];
	delete[] pTrans;

}