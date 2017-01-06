#include <cv.h>
#include "highgui.h"
#include "cxcore.h"
#include <stdlib.h>
#include <iostream>

#pragma comment(lib,"cv.lib")
#pragma comment(lib,"highgui.lib")
#pragma comment(lib,"cxcore.lib")


/*  
	Define : power 2 of A
*/
#define power2(A) \
	( (A) * (A) )

/*
	Define : power 3 of A
*/
#define power3(A) \
	( (A) * (A) * (A) )


/*  Define : Sign of a 
*/
#define Signof(a) (a > 0 ? 1.0 : -1.0)


/* 
	Cross product operation : R = A x B .  R is the result of A x B. R, A, and B are vectors with 3 elements.
	A, B, and R must have 3 allocated memory elements, respectively.
*/
#define CDOT(R,A,B)\
  ( (R)[0] = ( ((A)[1]*(B)[2]) - ((B)[1]*(A)[2]) ) ,\
    (R)[1] = ( ((A)[2]*(B)[0]) - ((A)[0]*(B)[2]) ) ,\
    (R)[2] = ( ((A)[0]*(B)[1]) - ((A)[1]*(B)[0]) ) )


/*
	Inner product of two vectors with 2 elements.  Returned value is inner product of vectors A and B
*/
#define DOT2(A,B) ( ((A)[0]*(B)[0]) + ((A)[1]*(B)[1]) )


/*
	Inner product of two vectors with 3 elements.  Returned value is inner product of vectors A and B
*/
#define DOT3(A,B) ( ((A)[0]*(B)[0]) + ((A)[1]*(B)[1]) + ((A)[2]*(B)[2]) )


/* 
	Inner product of two vectors with 4 elements.  Returned value is inner product of vectors A and B
*/
#define DOT4(A,B) ( ((A)[0]*(B)[0]) + ((A)[1]*(B)[1]) + ((A)[2]*(B)[2]) + ((A)[3]*(B)[3]) )



/*
	Summation first num elements in array A.
	return value is  : A[0] + A[1] + ... + A[num-1] 
*/
double SumOfList(double *A,int num);

/*
	Copy elements.  Copy num elements from source to res
*/
void copy_element(double *src,double *dst,int num);



/*
	Set one value "a" to pMatrix at (iRow,iCol)
*/
void CvMat_setdb(CvMat *pMatrix,int iRow,int iCol,double A);


/*
	Get one value from pMatrix at (iRow,iCol)
*/
double CvMat_getdb(CvMat *pMatrix,int iRow,int iCol);


/*
	Print out pMatrix
*/
void CvMat_printdb( FILE *pFileOut, char* format, CvMat *pMatrix );
//void cvQR(CvMat *inputA,CvMat *q,CvMat *r);

void cvMat_print( FILE *pFileOut, char* format, CvMat *pMatrix);
void cvMat_print( FILE *pFileOut, char* format, CvMat *pMatrix, int Mode);

void E2Rt(CvMat* matE,CvMat** R,CvMat** t);

//an easier multiply function
CvMat* sfmMatMul(CvMat* src1,CvMat* src2);
CvMat* sfmMatMul(CvMat* src1,CvMat* src2,CvMat* src3);
CvMat* sfmMatMul(CvMat* src1,double k);
CvMat* sfmMatMul(double k,CvMat* src1);

//an easier transpose function
CvMat* sfmT(CvMat* src);
//Align the matrix in horizontal direction
CvMat* sfmAlignMatH(CvMat* src1,CvMat* src2);

//Get a corresponding column
CvMat* sfmGetCols(CvMat* src,int startc,int endc);
CvMat* sfmGetCols(CvMat* src,int startc);

//Get a corresponding row
CvMat* sfmGetRows(CvMat* src, int startr,int endr);
CvMat* sfmGetRows(CvMat* src, int startc);

//Add two matrix
CvMat* sfmAdd(CvMat* src1,CvMat* src2);

CvMat* sfmSub(CvMat* src1,CvMat* src2);
//set the intrinsic matrix K
void SetK(double intriK[3][3],CvMat* matK);



//Mapping Matrix for two 3D clusters, min-error estimated
double Mapping_Matrix(double *Ro,double **Ox,double **Op,unsigned int size);

//Multiply a 4x4 matrix Trans to a 4x4 matrix R, and save as R
void Mult_Matrix(double  *Trans,double  *R);

//Set the 4x4 Matrix M as one Identity matrix;
void Iden_Matrix(double *M);

//Deterime the eigenvalue by Power Eigen Method.
void PowerEigen(int n,double *Eigen,double *A,double Tor);

//Apply one Matrix A to one Vector v, and get Result vector. n is the vector dimension.
void Mul_Matrix_v(int n,double *Result,double *A,double *v);

//return vector(A) lenght. A is one 3-elements vecotor.
double vector3L(double *A);


//Put matrix vertically in a dst matrix( mainly used in Resid in vgg_X_from_xP_nonlin.cpp)
//k is the starting row
//k starting from 0;
int PutMatV(CvMat* inputM,CvMat* dst, int k); 

//Put matrix in horizontal way
int PutMatH(CvMat* inputM,CvMat* dst, int k);

//Set ones in the matrix;
void SetOnes(CvMat* inMat);

//Release Matrix Group
//Release Dynamic Group
void cvReleaseMatGrp(CvMat** inMat,int K);
//Release Static Group
void cvReleaseMatStaGrp(CvMat** inMat,int K);

// convert rotation function to quaternion
CvMat* rot2q(CvMat *matRot);

//project matrix to qt
void projmat2qt(CvMat* matProj,CvMat* intK, CvMat*q, CvMat* t);






