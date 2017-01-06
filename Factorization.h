#if !defined(__FACTORIZATION___H___TH___)
#define __FACTORIZATION___H___TH___

#include <cv.h>
#include "highgui.h"
#include "cxcore.h"
#include "cv_util.h"

typedef class PERSPMATRIX PERSPMATRIX ;
typedef class FACTORIZATION FACTORIZATION ;
typedef class IMGBEHAVIOR IMGBEHAVIOR ;
typedef class USVLIST USVLIST;

/*  
	Purpose : Used only for FACTORIZATION 
	Normalize each column of matrix A
*/
int NormalizeMatrixColumn(CvMat *A);

/*  
	Purpose : Used only for FACTORIZATION 
	Assign symmetrical matrix for M34
*/
int SymmetricAssign(CvMat *M34,CvMat *M10,int index1,int index2);

/*  
	Purpose : Used only for FACTORIZATION 
	Special purpose for L-U decomposition and Unify diagonal elements.
*/
int CvMat_TLU(CvMat *L,CvMat *U,CvMat *A);


/*  
	Purpose : Used only for FACTORIZATION 
	Special purpose for convience issue of matrix multiply
*/
void matrix_apply_vector2D(double *res,double *M33,double *A);

/*
	This class is used for assistance for class FACTORIZATION
*/
class PERSPMATRIX
{
	public:
		double data[3][4];
		PERSPMATRIX() 
		{   
			for (int i=0;i<3;i++)
				for (int j=0;j<4;j++)
					data[i][j] = 0;
		}
};

/*
	This class is used for assistance for class FACTORIZATION
*/
class USVLIST   //	3nImg x nImg
{
	public:
		CvMat *Q,*U,*V;
		CvMat *S;
		USVLIST()  
		{
			
		}
		void Create(int t1,int t2) //t1 must be greater than t2
		{
			Q = cvCreateMat(t1,t2,CV_64FC1);
			U = cvCreateMat(t1,t1,CV_64FC1);
			V = cvCreateMat(t2,t2,CV_64FC1);
			S = cvCreateMat(t1,t2,CV_64FC1); //eigenvector
		}
		~USVLIST()
		{
			cvReleaseMat(&Q);
			cvReleaseMat(&U);
			cvReleaseMat(&V);
			cvReleaseMat(&S);
		}
};

/*
	This class is used for assistance for class FACTORIZATION
*/
class IMGBEHAVIOR
{
	public:
		double centroid[2];
		CvMat *H,*finM,*tmpM,*preTrans;
		IMGBEHAVIOR()  {	H = cvCreateMat(3,3,CV_64FC1); 
							preTrans = cvCreateMat(3,3,CV_64FC1);
							finM = cvCreateMat(3,4,CV_64FC1); 
							tmpM = cvCreateMat(3,4,CV_64FC1);  }
		~IMGBEHAVIOR() {  cvReleaseMat(&H); 
		                  cvReleaseMat(&finM);
		                  cvReleaseMat(&tmpM);   
		                  cvReleaseMat(&preTrans);   }
		                 
		

};


/*
	The Main Class: we will use it for handling data and factorization.
*/
class FACTORIZATION
{
	public:
		//input
		int nImg,nFeature;
		double ***data;  //[2][5][0] -->indicates the 3th img, 6th feature's u coordinate
		double intK[3][3];
		
		//output
		double **p3D;
		PERSPMATRIX *perspMatrix;
		double **CamPos;   //nImgx16 matrix
		
		//
		CvMat *intrinsicK; 
		IMGBEHAVIOR *imgBe;
		
		//function	
		FACTORIZATION(int numOfimg,int numOfFreat);
		double Execute(int nIterations);
		void SetCondition();
		void SaveToSFM(char *fileName);
		void ApplyMapping(double *M44);
		void ApplyTranslating(double *T);
		void ApplyScaling(double Sfactor);
		void Release();

};


#endif
