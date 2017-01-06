
#include "KLTracker.h"
#include <stdlib.h>
#include <stdio.h>
#include "pnmio.h"
#include "klt.h"

FeatureVector *KLTFeatureRetrievalRGB(unsigned char **RGBArray,int nCols,int nRows, int nFrames,int nFeatures)
{
  unsigned char *img1, *img2,**GreyArray=NULL;
  FeatureVector *host_feature;
  int i,j;

  KLT_TrackingContext tc;
  KLT_FeatureList fl;
  KLT_FeatureTable ft;

  if (nFrames < 2) return NULL;
  if (nFeatures < 3) return NULL;

  host_feature = new FeatureVector[nFrames];
  for (i=0;i<nFrames;i++) host_feature[i].Create(nFeatures);

  GreyArray = new unsigned char *[nFrames];
  for (i=0;i<nFrames;i++)
  {
	  GreyArray[i] = new unsigned char[nCols*nRows];
  }

  for (i=0;i<nFrames;i++)
	  _RGB_ToGray(RGBArray[i],GreyArray[i],nCols*nRows);

  img1 = &(GreyArray[0][0]);

  tc = KLTCreateTrackingContext();
  fl = KLTCreateFeatureList(nFeatures);
  ft = KLTCreateFeatureTable(nFrames, nFeatures);
  tc->sequentialMode = TRUE;
  tc->writeInternalImages = FALSE;
  tc->affineConsistencyCheck = -1;  /* set this to 2 to turn on affine consistency check */

  KLTSelectGoodFeatures(tc, img1, nCols, nRows, fl);
  KLTStoreFeatureList(fl, ft, 0);

//  KLTWriteFeatureListToPPM(fl, img1, nCols, nRows, "feat0.ppm");

  for (i = 1 ; i < nFrames ; i++)  {
//    sprintf(fnamein, "img%d.pgm", i);
	img2 = &(GreyArray[i][0]);
    KLTTrackFeatures(tc, img1, img2, nCols, nRows, fl);
//    KLTReplaceLostFeatures(tc, img2, ncols, nrows, fl);
    KLTStoreFeatureList(fl, ft, i);
//    sprintf(fnameout, "feat%d.ppm", i);
//    KLTWriteFeatureListToPPM(fl, img2, nCols, nRows, fnameout);
  }
//  KLTWriteFeatureTable(ft, "features.txt", "%5.1f");
//  KLTWriteFeatureTable(ft, "features.ft", NULL);

    for (j = 0 ; j < ft->nFeatures ; j++)  
	{
      for (i = 0 ; i < ft->nFrames ; i++)
	  {
		host_feature[i].vertex[j][0] = ft->feature[j][i]->x;
		host_feature[i].vertex[j][1] = ft->feature[j][i]->y;
		if (ft->feature[j][i]->val>=0) host_feature[i].isValid[j] = 1;
			else host_feature[i].isValid[j] = ft->feature[j][i]->val;
		
	  }
    }  

  KLTFreeFeatureTable(ft);
  KLTFreeFeatureList(fl);
  KLTFreeTrackingContext(tc);
  
  for (i=0;i<nFrames;i++)
	delete[] GreyArray[i];
  delete[] GreyArray;

  return host_feature;
}


FeatureVector *KLTFeatureRetrievalGrey(unsigned char **GreyArray,int nCols,int nRows, int nFrames,int nFeatures)
{
  unsigned char *img1, *img2;
  FeatureVector *host_feature;
  int i,j;

  KLT_TrackingContext tc;
  KLT_FeatureList fl;
  KLT_FeatureTable ft;

  if (nFrames < 2) return NULL;
  if (nFeatures < 3) return NULL;

  host_feature = new FeatureVector[nFrames];
  for (i=0;i<nFrames;i++) host_feature[i].Create(nFeatures);

  img1 = &(GreyArray[0][0]);

  tc = KLTCreateTrackingContext();
  fl = KLTCreateFeatureList(nFeatures);
  ft = KLTCreateFeatureTable(nFrames, nFeatures);

  tc->sequentialMode = TRUE;
  tc->writeInternalImages = FALSE;
  tc->affineConsistencyCheck = -1;  /* set this to 2 to turn on affine consistency check */

  KLTSelectGoodFeatures(tc, img1, nCols, nRows, fl);
  KLTStoreFeatureList(fl, ft, 0);

//  KLTWriteFeatureListToPPM(fl, img1, nCols, nRows, "feat0.ppm");

  for (i = 1 ; i < nFrames ; i++)  {
//    sprintf(fnamein, "img%d.pgm", i);
	img2 = &(GreyArray[i][0]);
    KLTTrackFeatures(tc, img1, img2, nCols, nRows, fl);
//    KLTReplaceLostFeatures(tc, img2, ncols, nrows, fl);
    KLTStoreFeatureList(fl, ft, i);
//    sprintf(fnameout, "feat%d.ppm", i);
//    KLTWriteFeatureListToPPM(fl, img2, nCols, nRows, fnameout);
  }
//  KLTWriteFeatureTable(ft, "features.txt", "%5.1f");
//  KLTWriteFeatureTable(ft, "features.ft", NULL);

    for (j = 0 ; j < ft->nFeatures ; j++)  
	{
      for (i = 0 ; i < ft->nFrames ; i++)
	  {
		host_feature[i].vertex[j][0] = ft->feature[j][i]->x;
		host_feature[i].vertex[j][1] = ft->feature[j][i]->y;
		if (ft->feature[j][i]->val>=0) host_feature[i].isValid[j] = 1;
			else host_feature[i].isValid[j] = ft->feature[j][i]->val;
		
	  }
    }  

  KLTFreeFeatureTable(ft);
  KLTFreeFeatureList(fl);
  KLTFreeTrackingContext(tc);
  
  return host_feature;
}


FeatureVector *KLTFeatureRetrievalList(IMGList *RGBImgList,int nCols,int nRows, int nFrames,int nFeatures)
{
  unsigned char *img1, *img2;
  IMGList *pImg,*GreyArray=NULL,**GreyArryTable;
  FeatureVector *host_feature;
  int i,j;

  KLT_TrackingContext tc;
  KLT_FeatureList fl;
  KLT_FeatureTable ft;

  if (nFrames < 2) return NULL;
  if (nFeatures < 3) return NULL;
  if (!RGBImgList) return NULL;
  

  host_feature = new FeatureVector[nFrames];
  for (i=0;i<nFrames;i++) host_feature[i].Create(nFeatures);


  GreyArryTable = new IMGList*[nFrames];

  pImg = RGBImgList;
  for (i=0;i<nFrames;i++)
  {
	  GreyArray = new IMGList(nCols,nRows,1);
	  GreyArryTable[i] = GreyArray;
	  _RGB_ToGray(pImg->Img,GreyArryTable[i]->Img,nCols*nRows);
	  GO_NEXT(pImg);
  }

  img1 = GreyArryTable[0]->Img;

  tc = KLTCreateTrackingContext();
  fl = KLTCreateFeatureList(nFeatures);
  ft = KLTCreateFeatureTable(nFrames, nFeatures);
  tc->sequentialMode = TRUE;
  tc->writeInternalImages = FALSE;
  tc->affineConsistencyCheck = -1;  /* set this to 2 to turn on affine consistency check */

  KLTSelectGoodFeatures(tc, img1, nCols, nRows, fl);
  KLTStoreFeatureList(fl, ft, 0);

//  KLTWriteFeatureListToPPM(fl, img1, nCols, nRows, "feat0.ppm");

  for (i = 1 ; i < nFrames ; i++)  {
//    sprintf(fnamein, "img%d.pgm", i);
	img2 = GreyArryTable[i]->Img;;
    KLTTrackFeatures(tc, img1, img2, nCols, nRows, fl);
//    KLTReplaceLostFeatures(tc, img2, ncols, nrows, fl);
    KLTStoreFeatureList(fl, ft, i);
//    sprintf(fnameout, "feat%d.ppm", i);
//    KLTWriteFeatureListToPPM(fl, img2, nCols, nRows, fnameout);
  }
//  KLTWriteFeatureTable(ft, "features.txt", "%5.1f");
//  KLTWriteFeatureTable(ft, "features.ft", NULL);

    for (j = 0 ; j < ft->nFeatures ; j++)  
	{
      for (i = 0 ; i < ft->nFrames ; i++)
	  {
		host_feature[i].vertex[j][0] = ft->feature[j][i]->x;
		host_feature[i].vertex[j][1] = ft->feature[j][i]->y;
		if (ft->feature[j][i]->val>=0) host_feature[i].isValid[j] = 1;
			else host_feature[i].isValid[j] = ft->feature[j][i]->val;
		
	  }
    }  

  KLTFreeFeatureTable(ft);
  KLTFreeFeatureList(fl);
  KLTFreeTrackingContext(tc);

  for (i=0;i<nFrames;i++)
  {
	  pImg = GreyArryTable[i];
	  delete pImg;
  }
  delete[] GreyArryTable;

  return host_feature;

}


FeatureVector *KLTFeatureRetrievalListMin(IMGList *RGBImgList,int nCols,int nRows, int nFrames,int nFeatures,int nMinFeatures)
{
  unsigned char *img1, *img2;
  IMGList *pImg,*GreyArray=NULL,**GreyArryTable;
  FeatureVector *host_feature;
  int i,j,nFeatCount,maxID;

  KLT_TrackingContext tc;
  KLT_FeatureList fl,fl_pre;
  KLT_FeatureTable ft;

  if (nFrames < 2) return NULL;
  if (nFeatures < 3) return NULL;
  if (!RGBImgList) return NULL;
  if (nFeatures<nMinFeatures) return NULL;


  host_feature = new FeatureVector[nFrames];
  for (i=0;i<nFrames;i++) host_feature[i].Create(nFeatures);

  GreyArryTable = new IMGList*[nFrames];

  pImg = RGBImgList;
  for (i=0;i<nFrames;i++)
  {
	  GreyArray = new IMGList(nCols,nRows,1);
	  GreyArryTable[i] = GreyArray;
	  _RGB_ToGray(pImg->Img,GreyArryTable[i]->Img,nCols*nRows);
	  GO_NEXT(pImg);
  }


  tc = KLTCreateTrackingContext();
  fl = KLTCreateFeatureList(nFeatures);
  fl_pre = KLTCreateFeatureList(nFeatures);
  ft = KLTCreateFeatureTable(nFrames, nFeatures);
	  
  	  img1 = GreyArryTable[0]->Img;

	  tc->sequentialMode = TRUE;
	  tc->writeInternalImages = FALSE;
	  tc->affineConsistencyCheck = -1;  /* set this to 2 to turn on affine consistency check */

	  KLTSelectGoodFeatures(tc, img1, nCols, nRows, fl);
	  KLTStoreFeatureList(fl, ft, 0);

	  for (i = 1; i < nFrames ; i++)  
	  {
		img2 = GreyArryTable[i]->Img;
		
		printf("Tracking %dth frame\n",i);
		KLTTrackFeatures(tc, img1, img2, nCols, nRows, fl);
		
		nFeatCount = 0;
		for (j=0;j<fl->nFeatures;j++)
			if (fl->feature[j]->val>=0) nFeatCount++;

		if (nFeatCount<nMinFeatures)
		{
			for (j=0;j<fl->nFeatures;j++)
				if (fl->feature[j]->val<0) 
					host_feature[i].isValid[j] = 13083;

			KLTReplaceLostFeatures(tc, img2, nCols, nRows, fl);
			KLTStoreFeatureList(fl, ft, i);
		}
		else
			KLTStoreFeatureList(fl, ft, i);
	  }

	  maxID = 0;
	  for (j = 0 ; j < ft->nFeatures ; j++)  
	  {
		  for (i=0;i<nFrames;i++)
		  {
				host_feature[i].vertex[j][0] = ft->feature[j][i]->x;
				host_feature[i].vertex[j][1] = ft->feature[j][i]->y;
				if (host_feature[i].isValid[j] ==13083) maxID++;
				if (ft->feature[j][i]->val>=0) 
				{
					host_feature[i].isValid[j] = maxID;
					if (i==nFrames-1)  {
						maxID++; 
						printf("Boundary ID = %d\n",maxID);
						continue;}
				}
				else 
				{
					host_feature[i].isValid[j] = ft->feature[j][i]->val;
					if (i>0)
					{
						if (ft->feature[j][i-1]->val>=0)
						{
								maxID++;
								printf("Internal ID = %d\n",maxID);
						}
					}
				}
		  }
	  }

	  KLTFreeFeatureTable(ft);
	  KLTFreeFeatureList(fl);
	  KLTFreeFeatureList(fl_pre);
	  KLTFreeTrackingContext(tc);

  for (i=0;i<nFrames;i++)
  {
	  pImg = GreyArryTable[i];
	  delete pImg;
  }
  delete[] GreyArryTable;

  return host_feature;
}
