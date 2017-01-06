#include "cv_util.h"

using namespace std;

inline double sign(double a)
{
    return (a > 0 ? 1.0 : -1.0);
}
void cvMat_print( FILE *pFileOut, char* format, CvMat *pMatrix) 
{
	cvMat_print(pFileOut,format,pMatrix,0);
}


void cvMat_print( FILE *pFileOut, char* format, CvMat *pMatrix,int Mod) 
{
	
    int iRow, iCol;
    int iRowStep = pMatrix->step / sizeof( double );

	if(Mod==0)
	{
		for ( iRow = 0; iRow < pMatrix->rows; iRow++ ) 
		{
			for ( iCol = 0; iCol < pMatrix->cols; iCol++ ) 
			{
				fprintf( pFileOut, format,pMatrix->data.db[ iRow * iRowStep + iCol ] );
			}
			fprintf( pFileOut, "\n" );
		}
		fprintf(pFileOut,"\n");
	}
	else if(Mod==1)//print line
	{
		for ( iRow = 0; iRow < pMatrix->rows; iRow++ ) 
		{
			for ( iCol = 0; iCol < pMatrix->cols; iCol++ ) 
			{
				fprintf( pFileOut, format,pMatrix->data.db[ iRow * iRowStep + iCol ] );
			}			
		}

	}
	else if(Mod==2)
	{
		for ( iRow = 0; iRow < pMatrix->rows; iRow++ ) 
		{
			for ( iCol = 0; iCol < pMatrix->cols; iCol++ ) 
			{
				fprintf( pFileOut, format,pMatrix->data.db[ iRow * iRowStep + iCol ] );
			}
			fprintf( pFileOut, "\n" );
		}
	}
}
void SetK(double intriK[3][3],CvMat* matK)
{
	int i,j,k;
	k=0;
	for(i=0;i<=2;i++)
	{
		for(j=0;j<=2;j++)
		{
			matK->data.db[k]=intriK[i][j];
			k++;
		}
	}	
}
void E2Rt(CvMat* matE,CvMat** R,CvMat** t)
{
	CvMat* U;
	CvMat* D;
	CvMat* V;
	U= cvCreateMat(3,3,CV_64FC1);
	V= cvCreateMat(3,3,CV_64FC1);
	D= cvCreateMat(3,3,CV_64FC1);

	//Use SVD to get U,D,V
	cvSVD( matE, D, U, V);
	/*
	cvMat_print(stdout,"%7.3f",U);
	cvMat_print(stdout,"%7.3f",D);
	cvMat_print(stdout,"%7.3f",V);
*/
	CvMat* W;
	W=cvCreateMat(3,3,CV_64FC1);
	double dblTemp[3][3]={ {0,-1,0},{1,0,0},{0,0,1}};
	SetK(dblTemp,W);

	CvMat* tV;
	CvMat* tW;
	//Get R
	tV=sfmT(V);
	tW=sfmT(W);

	R[0]=sfmMatMul(U,W,tV);
	R[1]=sfmMatMul(U,W,tV);
	//cvCopy(R[0],R[1]);
	R[2]=sfmMatMul(U,tW,tV);
	R[3]=sfmMatMul(U,tW,tV);
	//cvCopy(R[2],R[3]);

	
	CvMat* tR;
	int i;
	for(i=0;i<3;i++)
	{
		tR=sfmMatMul(R[i],sign(cvDet(R[i])));
		cvCopy(tR,R[i]);
		//R[i]=sfmMatMul(R[i],sign(cvDet(R[i])));
		cvReleaseMat(&tR);
	}

	//Get t from U(:,3);
	//cvMat_print(stdout,"%7.3f",U);
	t[0]=sfmGetCols(U,2);
	//cvMat_print(stdout,"%7.3f",t[0]);
	t[2]=sfmGetCols(U,2);
	//cvCopy(t[0],t[2]);
	t[1]=sfmMatMul(-1.0,t[0]);
	//cvCopy(t[1],t[3]);
	t[3]=sfmMatMul(-1.0,t[0]);

	cvReleaseMat(&U);
	cvReleaseMat(&V);
	cvReleaseMat(&D);
	cvReleaseMat(&W);
	cvReleaseMat(&tV);
	cvReleaseMat(&tW);

	

}
CvMat* sfmGetRows(CvMat* src, int startr,int endr)
{
	CvMat* dst;
	dst=cvCreateMat(endr-startr+1,src->cols,src->type);
	int i,j;
	int k=0;
	for(i=startr;i<=endr;i++)
	{
		for(j=0;j<src->cols;j++)
		{
			dst->data.db[k]=src->data.db[i*src->cols+j];
			k++;
		}
	}
	return dst;

}
CvMat* sfmGetRows(CvMat* src, int startr)
{
	return sfmGetRows(src,startr,startr);
}

CvMat* sfmGetCols(CvMat* src,int startc,int endc)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat(src->rows,endc-startc+1,src->type);
	int i,j;
	int k=0;
	for(i=0;i<src->rows;i++)
	{
		for(j=startc;j<=endc;j++)
		{
			dst->data.db[k]=src->data.db[i*src->cols+j];
			k++;
		}
	}
	return dst;
}
CvMat* sfmGetCols(CvMat* src,int startc)
{
	return sfmGetCols(src,startc,startc);
}
CvMat* sfmMatMul(CvMat* src1,CvMat* src2)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat(src1->rows,src2->cols,src1->type);
	cvMatMul(src1,src2,dst);
	return dst;
}
CvMat* sfmMatMul(CvMat* src1,CvMat* src2,CvMat* src3)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat(src1->rows,src3->cols,src1->type);
	cvMatMul(src1,sfmMatMul(src2,src3),dst);
	return dst;
}
CvMat* sfmMatMul(CvMat* src1,double k)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat(src1->rows,src1->cols,src1->type);
	cvCopy(src1,dst);
	int i,j,m;
	m=0;
	for(i=0;i<src1->rows;i++)
		for(j=0;j<src1->cols;j++)
		{
			dst->data.db[m]=dst->data.db[m]*k;
			m++;
		}
	return dst;
}
CvMat* sfmMatMul(double k,CvMat* src1)
{
	return sfmMatMul(src1,k);
}
CvMat* sfmT(CvMat* src)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat(src->cols,src->rows,src->type);
	cvTranspose(src,dst);
	return dst;
}
CvMat* sfmAlignMatH(CvMat* src1,CvMat* src2)
{
	CvMat* dst;//=new CvMat;
	int maxcol=src2->cols+src1->cols;
	dst=cvCreateMat( src1->rows,maxcol,src1->type);
	
	int i,j;
	
	int k=0;
	int l=0;
	for(i=0;i<src1->rows;i++)
	{
		for(j=0;j<src1->cols;j++)
		{
			dst->data.db[i*maxcol+j]=src1->data.db[k];
			k++;
		}
		for(j=src1->cols;j<maxcol;j++)
		{
			dst->data.db[i*maxcol+j]=src2->data.db[l];
			l++;
		}
	}
	return dst;
}
CvMat* sfmAdd(CvMat* src1,CvMat* src2)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat( src1->rows,src1->cols,src1->type);
	cvAdd(src1,src2,dst);
	return dst;
}
CvMat* sfmSub(CvMat* src1,CvMat* src2)
{
	CvMat* dst;//=new CvMat;
	dst=cvCreateMat( src1->rows,src1->cols,src1->type);
	cvSub(src1,src2,dst);
	return dst;

}

double SumOfList(double *A,int num)
{
	double tmp=0;
	for (int i=0;i<num;i++)
		tmp += A[i];
	return tmp;
}


void copy_element(double *src,double *dst,int num)
{
	for (int i=0;i<num;i++)	dst[i] = src[i];
}

void CvMat_setdb(CvMat *pMatrix,int iRow,int iCol,double A)
{
	pMatrix->data.db[iRow * pMatrix->cols + iCol] = A;
}

double CvMat_getdb(CvMat *pMatrix,int iRow,int iCol)
{
	return pMatrix->data.db[iRow * pMatrix->cols + iCol];
}

void CvMat_printdb( FILE *pFileOut, char* format, CvMat *pMatrix ) 
{
    int iRow, iCol;
    int iRowStep = pMatrix->step / sizeof( double );

    for ( iRow = 0; iRow < pMatrix->rows; iRow++ ) 
	{
        for ( iCol = 0; iCol < pMatrix->cols; iCol++ ) 
		{
            fprintf( pFileOut, format,pMatrix->data.db[ iRow * iRowStep + iCol ] );
        }
        fprintf( pFileOut, "\n" );
    }
}


double Mapping_Matrix(double *Ro,double **Ox,double **Op,unsigned int size)
{
  unsigned int i,j,k,l;
  double mup[3],mux[3],Covar[9],Qmatrix[16],Ro_M[9],qt[3],MSR[3];
  double q[4];

//======Initial Zero========================
  for (i=0;i<3;i++)  { mup[i]=0; mux[i]=0; MSR[i]=0; }
  for (i=0;i<9;i++)  Covar[i]=0;

//======To Find the center of Mass==========
  for (i=0;i<size;i++)
        for (j=0;j<3;j++)
              {  mux[j] += Ox[i][j];
                 mup[j] += Op[i][j]; }

  for (i=0;i<3;i++)
  { mup[i]=mup[i]/size;   mux[i]=mux[i]/size;}

//==========Setup Cross covariance=============
  for (i=0;i<size;i++)
  for (j=0;j<3;j++)
  for (k=0;k<3;k++)
  Covar[3*j+k] += (Op[i][j]-mup[j])*(Ox[i][k]-mux[k]);

  for (i=0;i<9;i++) Covar[i]=Covar[i]/size;

//===========Setup Q matrix===========================
  Qmatrix[0]=Covar[0]+Covar[4]+Covar[8];
  Qmatrix[1]=Covar[5]-Covar[7];
  Qmatrix[2]=-(Covar[2]-Covar[6]);
  Qmatrix[3]=Covar[1]-Covar[3];
  Qmatrix[4]=Qmatrix[1];
  Qmatrix[8]=Qmatrix[2];
  Qmatrix[12]=Qmatrix[3];
  Qmatrix[5]=2*Covar[0]-Qmatrix[0];
  Qmatrix[10]=2*Covar[4]-Qmatrix[0];
  Qmatrix[15]=2*Covar[8]-Qmatrix[0];
  Qmatrix[6]=Covar[1]+Covar[3];
  Qmatrix[7]=Covar[2]+Covar[6];
  Qmatrix[11]=Covar[5]+Covar[7];
  Qmatrix[9]=Qmatrix[6];
  Qmatrix[13]=Qmatrix[7];
  Qmatrix[14]=Qmatrix[11];

//============Solution of EigneValue==============
  PowerEigen(4,q,Qmatrix,1e-8);
//===========Sort the Rotating Matrix=============

  Ro_M[0]= q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
  Ro_M[1]= 2*( q[1]*q[2] - q[0]*q[3] );
  Ro_M[2]= 2*( q[1]*q[3] + q[0]*q[2] );
  Ro_M[3]= 2*( q[1]*q[2] + q[0]*q[3] );
  Ro_M[4]= q[0]*q[0] + q[2]*q[2] - q[1]*q[1] - q[3]*q[3];
  Ro_M[5]= 2*( q[2]*q[3] - q[0]*q[1] );
  Ro_M[6]= 2*( q[1]*q[3] - q[0]*q[2] );
  Ro_M[7]= 2*( q[2]*q[3] + q[0]*q[1] );
  Ro_M[8]= q[0]*q[0] + q[3]*q[3] - q[1]*q[1] - q[2]*q[2];

  qt[0]=mux[0]-(Ro_M[0]*mup[0]+Ro_M[1]*mup[1]+Ro_M[2]*mup[2]);
  qt[1]=mux[1]-(Ro_M[3]*mup[0]+Ro_M[4]*mup[1]+Ro_M[5]*mup[2]);
  qt[2]=mux[2]-(Ro_M[6]*mup[0]+Ro_M[7]*mup[1]+Ro_M[8]*mup[2]);

//========Sorting Rotating Matrix====================
  Iden_Matrix(Ro);

  for (i=0;i<3;i++)
  for (j=0;j<3;j++) Ro[4*i+j]=Ro_M[3*i+j]; //Copy Rotation

  Iden_Matrix(Qmatrix);
  for (i=0;i<3;i++) Qmatrix[4*i+3] = +qt[i];
  Mult_Matrix(Qmatrix,Ro);  //==Translation

//========Mean Square Error==========================
  for (i=0;i<size;i++)
  for (j=0;j<3;j++)
        MSR[j] += power2( (Ox[i][j] - DOT3(&(Ro[j*4]),Op[i]) ));

  MSR[0] /= float(size);
  MSR[1] /= float(size);
  MSR[2] /= float(size);

  if (MSR[0]+MSR[1]+MSR[2]<-1e-8) return -1;
  return sqrt(MSR[0]+MSR[1]+MSR[2]);
}


void Mult_Matrix(double  *Trans,double  *R)
{
   int i,j,k;
   double temp[16];
   for (i=0;i<16;i++) temp[i]=0;

   for (i = 0 ; i < 4 ; i++ )
   for (j = 0 ; j < 4 ; j++ )
   for (k = 0 ; k < 4 ; k++ )
   temp[4*i+j] += Trans[4*i+k]*R[4*k+j];

   for ( k = 0 ; k < 16 ; k++ )
   R[k] = temp[k];
}

//Set the 4x4 Matrix M as one Identity matrix;
void Iden_Matrix(double *M)
{
   M[0] = 1;  M[1] = 0;  M[2] = 0;  M[3] = 0;
   M[4] = 0;  M[5] = 1;  M[6] = 0;  M[7] = 0;
   M[8] = 0;  M[9] = 0;  M[10]= 1;  M[11] = 0;
   M[12]= 0;  M[13]= 0;  M[14]= 0;  M[15] = 1;
}

//Deterime the eigenvalue by Power Eigen Method.
void PowerEigen(int n,double *Eigen,double *A,double Tor)
{
  int i,j;
  double x[1024],temp=1;

  for (i=0;i<n;i++) x[i]=2*(0.5-i%2);
  Mul_Matrix_v(n,Eigen,A,x);

  i=0;
  while ( (i<1024) && (temp > Tor )  )
  {
   for (j=0;j<n;j++) x[j]=Eigen[j];
   Mul_Matrix_v(n,Eigen,A,x);
   temp=0;   for (j=0;j<n;j++) temp+=fabs(x[j]-Eigen[j]);
   i++; }
  if (Eigen[0]<0) for (i=0;i<n;i++) Eigen[i]=-Eigen[i];
}


//Apply one Matrix A to one Vector v, and get Result vector. n is the vector dimension.
void Mul_Matrix_v(int n,double *Result,double *A,double *v)
{
  int i,j;
  double temp;
  for (i=0;i<n;i++) Result[i]=0.0;

  for (i=0;i<n;i++)
  for (j=0;j<n;j++)
      Result[i]+=A[n*i+j]*v[j];

  temp=0;
  for (i=0;i<n;i++) temp+=Result[i]*Result[i];
  for (i=0;i<n;i++) Result[i]=Result[i]/(sqrt(temp)*(1+1e-8));
}

//Determine Vector length 
double vector3L(double *A)
{
	double tmp = power2(A[0]) + power2(A[1]) + power2(A[2]);
	if (tmp<1e-12) return 0.0;
	return sqrt(tmp);
}

int PutMatV(CvMat* inputM,CvMat* dst, int k)
{
	if(dst->cols!=inputM->cols)
	{
		printf("the input Matrix and dst matrix should be of the same col\n");
		return 0;
	}
	int i,j;
	for(i=k;i<=k+inputM->rows-1;i++)
	{
		for(j=0;j<=inputM->cols-1;j++)
		{
			dst->data.db[i* inputM->cols + j]=inputM->data.db[(i-k) * inputM->cols + j];
		}
	}
	return 1;
}
int PutMatH(CvMat* inputM,CvMat* dst, int k)
{
	if(dst->rows!=inputM->rows)
	{
		printf("the input Matrix and dst matrix should be of the same rows\n");
		return 0;
	}
	int i,j;
	for(i=0;i<=inputM->rows-1;i++)
	{
		for(j=k;j<=k+inputM->cols-1;j++)
		{
			dst->data.db[i* dst->cols + j]=inputM->data.db[i* inputM->cols + j-k];
		}
	}
	return 1;
}

void SetOnes(CvMat* inMat)
{
	int i,j;
	for(i=0;i<inMat->rows;i++)
	{
		for(j=0;j< inMat->cols;j++)
		{
			inMat->data.db[i*inMat->cols+j]=1;
		}
	}
}

void cvReleaseMatGrp(CvMat** inMat,int K)
{
	int i;
	for(i=0;i<K;i++)
	{
		cvReleaseMat(&inMat[i]);
	}
	delete inMat;
}
void cvReleaseMatStaGrp(CvMat** inMat,int K)
{
	int i;
	for(i=0;i<K;i++)
	{
		cvReleaseMat(&inMat[i]);
	}
}
// convert rotation matrix to quaternion
// added by Utsav
CvMat* rot2q(CvMat *matRot)
{
	int iRowStep = matRot->step / sizeof( double );
	double t = 1 + matRot->data.db[0*iRowStep+0] + matRot->data.db[1*iRowStep+1] + matRot->data.db[2*iRowStep+2];
	CvMat *q = cvCreateMat(1, 4, CV_64FC1);
	if (t > pow((double)10, -10))
	{
		double s = cvSqrt(t) * 2;
		q->data.db[1] = (matRot->data.db[2*iRowStep+1] - matRot->data.db[1*iRowStep+2]) / s; //here
		q->data.db[2] = (matRot->data.db[0*iRowStep+2] - matRot->data.db[2*iRowStep+0]) / s;
		q->data.db[3] = (matRot->data.db[1*iRowStep+0] - matRot->data.db[0*iRowStep+1]) / s; 
		q->data.db[0] = 0.25 * s;
	}
	else if (matRot->data.db[0*iRowStep+0] > matRot->data.db[1*iRowStep+1] && matRot->data.db[0*iRowStep+0] > matRot->data.db[2*iRowStep+2]) 
	{
		double s = cvSqrt(1.0 + matRot->data.db[0*iRowStep+0] - matRot->data.db[1*iRowStep+1] - matRot->data.db[2*iRowStep+2])*2; 
		q->data.db[1] = 0.25 * s; 
		q->data.db[2] = (matRot->data.db[0*iRowStep+1] + matRot->data.db[1*iRowStep+0]) / s; 
		q->data.db[3] = (matRot->data.db[2*iRowStep+0] + matRot->data.db[0*iRowStep+2]) / s; 
		q->data.db[0] = (matRot->data.db[2*iRowStep+1] - matRot->data.db[1*iRowStep+2]) / s; 
	}
	else if (matRot->data.db[1*iRowStep+1] > matRot->data.db[2*iRowStep+2]) 
	{
	    double s = sqrt(1.0 + matRot->data.db[1*iRowStep+1] - matRot->data.db[0*iRowStep+0] - matRot->data.db[2*iRowStep+2])*2; 
	    q->data.db[1] = (matRot->data.db[0*iRowStep+1]+matRot->data.db[1*iRowStep+0]) / s; 
	    q->data.db[2] = 0.25 * s; 
	    q->data.db[3] = (matRot->data.db[1*iRowStep+2]+matRot->data.db[2*iRowStep+1]) / s; 
	    q->data.db[0] = (matRot->data.db[0*iRowStep+2]-matRot->data.db[2*iRowStep+0]) / s; 
	}
	else 
	{
		double s = sqrt(1.0 + matRot->data.db[2*iRowStep+2] - matRot->data.db[0*iRowStep+0] - matRot->data.db[1*iRowStep+1])*2; 
		q->data.db[1] = (matRot->data.db[2*iRowStep+0]+matRot->data.db[0*iRowStep+2]) / s; 
		q->data.db[2] = (matRot->data.db[1*iRowStep+2]+matRot->data.db[2*iRowStep+1]) / s; 
		q->data.db[3] = 0.25 * s;   
		q->data.db[0] = (matRot->data.db[1*iRowStep+0]-matRot->data.db[0*iRowStep+1]) / s; 
	}
	return q;
}

//projmat2qt
//do not have to initialize t and q;
void projmat2qt(CvMat* matProj,CvMat* intK, CvMat*q, CvMat* t)
{
	CvMat* tR;
	CvMat* ttR;
	CvMat* tI;
	
	CvMat* tt;
	CvMat* ttt;
	CvMat* qq;
	tI=cvCreateMat(intK->rows,intK->cols,intK->type);
	cvInvert(intK,tI);
	tR=sfmMatMul(tI,matProj);
	

	tt=sfmGetCols(tR,3);
	ttt=sfmT(tt);

	ttR=sfmGetCols(tR,0,2);
	/*
	//for debug
	printf("printting ttR\n");
	cvMat_print(stdout,"%7.3f ",ttR);
	*/
	qq=rot2q(ttR);
	
	/*
	//for debug
	printf("printting qq\n");
	cvMat_print(stdout,"%7.3f ",qq);
	*/

	cvCopy(ttt,t);
	cvCopy(qq,q);

	cvReleaseMat(&tR);
	cvReleaseMat(&ttR);
	cvReleaseMat(&tI);
	cvReleaseMat(&tt);
	cvReleaseMat(&qq);
	cvReleaseMat(&ttt);
		
}


