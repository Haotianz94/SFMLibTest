#include "Factorization.h"



void matrix_apply_vector2D(double *res,double *M33,double *A)
{
	res[0] = M33[0] * A[0] + M33[1] * A[1] + M33[2];
	res[1] = M33[3] * A[0] + M33[4] * A[1] + M33[5];
}

int CvMat_TLU(CvMat *L,CvMat *U,CvMat *A)
{
	int i,j,m=1,k,n;
	double tmp;
	
	if (A->rows!=A->cols) return -1;
	n = A->rows;
	for (k=1;k<n;k++)
	{
	    m = m+k-1;
	    if (CvMat_getdb(A,m-1,k-1) != 0.0)
	    {    
	        for (i=k+1;i<n+1;i++)
	        {
	        	tmp = CvMat_getdb(A,i-1,k-1)/CvMat_getdb(A,k-1,k-1);
	        	CvMat_setdb(A,i-1,k-1,tmp);
	        }
	        for (i=k+1;i<n+1;i++)
			for (j=k+1;j<n+1;j++)
	        {
	        	tmp = CvMat_getdb(A,i-1,j-1) - CvMat_getdb(A,i-1,k-1)*CvMat_getdb(A,k-1,j-1);
	        	CvMat_setdb(A,i-1,j-1,tmp); 
	        }
	    }
	}

	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			if (i>j)
				CvMat_setdb(L,i,j,CvMat_getdb(A,i,j));
			else if (i==j)
				CvMat_setdb(L,i,j,1);
			else
				CvMat_setdb(L,i,j,0);

	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			if (j>=i) 
				CvMat_setdb(U,i,j,CvMat_getdb(A,i,j));
			else
				CvMat_setdb(U,i,j,0);
	return 1;
}

int NormalizeMatrixColumn(CvMat *A)
{
	int i,j;
	double tmp,sign_dir;
	for (j=0;j<A->cols;j++)
	{
		tmp = 0;sign_dir=0;
		for (i=0;i<A->rows;i++)
		{
			tmp += power2(CvMat_getdb(A,i,j));
			if (i%3==2) sign_dir += CvMat_getdb(A,i,j);
		}
		tmp = sqrt(tmp);
		if (sign_dir>=0)
		{
			for (i=0;i<A->rows;i++)
				CvMat_setdb(A,i,j,CvMat_getdb(A,i,j)/tmp);
		}
		else
		{
			for (i=0;i<A->rows;i++)
				CvMat_setdb(A,i,j,-CvMat_getdb(A,i,j)/tmp);
		}
	}
	return 1;
}

int SymmetricAssign(CvMat *M34,CvMat *M10,int index1,int index2)
{
	M10->data.db[0] = CvMat_getdb(M34,index1,0)*CvMat_getdb(M34,index2,0); 
	              //m(1)=M(i,1)*M(j,1);
	M10->data.db[1] = CvMat_getdb(M34,index1,0)*CvMat_getdb(M34,index2,1)+CvMat_getdb(M34,index1,1)*CvMat_getdb(M34,index2,0);
		          //m(2)=M(i,1)*M(j,2)+M(i,2)*M(j,1);
	M10->data.db[2] = CvMat_getdb(M34,index1,0)*CvMat_getdb(M34,index2,2)+CvMat_getdb(M34,index1,2)*CvMat_getdb(M34,index2,0); 
		          //m(3)=M(i,1)*M(j,3)+M(i,3)*M(j,1);
	M10->data.db[3] = CvMat_getdb(M34,index1,0)*CvMat_getdb(M34,index2,3)+CvMat_getdb(M34,index1,3)*CvMat_getdb(M34,index2,0); 
		          //m(4)=M(i,1)*M(j,4)+M(i,4)*M(j,1);
	M10->data.db[4] = CvMat_getdb(M34,index1,1)*CvMat_getdb(M34,index2,1); 
		          //m(5)=M(i,2)*M(j,2);
	M10->data.db[5] = CvMat_getdb(M34,index1,1)*CvMat_getdb(M34,index2,2)+CvMat_getdb(M34,index1,2)*CvMat_getdb(M34,index2,1);
		          //m(6)=M(i,2)*M(j,3)+M(i,3)*M(j,2);
	M10->data.db[6] = CvMat_getdb(M34,index1,1)*CvMat_getdb(M34,index2,3)+CvMat_getdb(M34,index1,3)*CvMat_getdb(M34,index2,1);
		          //m(7)=M(i,2)*M(j,4)+M(i,4)*M(j,2);
	M10->data.db[7] = CvMat_getdb(M34,index1,2)*CvMat_getdb(M34,index2,2);
		          //m(8)=M(i,3)*M(j,3);
	M10->data.db[8] = CvMat_getdb(M34,index1,2)*CvMat_getdb(M34,index2,3)+CvMat_getdb(M34,index1,3)*CvMat_getdb(M34,index2,2);
		          //m(9)=M(i,3)*M(j,4)+M(i,4)*M(j,3);
	M10->data.db[9] = CvMat_getdb(M34,index1,3)*CvMat_getdb(M34,index2,3);
	              //m(10)=M(i,4)*M(j,4);
	return 1;
}

FACTORIZATION::FACTORIZATION(int numOfimg,int numOfFreat)
{
	int i,j;
	if (numOfimg<=0) numOfimg = 1;
	if (numOfFreat<=0) numOfFreat = 1;
	nImg = numOfimg;
	nFeature = numOfFreat;
	
	data = new double **[nImg];
	for (i=0;i<nImg;i++)
		data[i] = new double* [numOfFreat];
	
	for (i=0;i<nImg;i++)
	for (j=0;j<nFeature;j++)
		data[i][j] = new double[2];
				
	p3D = new double*[nFeature];
	for (j=0;j<nFeature;j++)
		p3D[j] = new double[3];
	
	perspMatrix = new PERSPMATRIX[nImg];
	imgBe = new IMGBEHAVIOR[nImg];
	CamPos = new double*[nImg];
	for (i=0;i<nImg;i++)
		CamPos[i] = new double[16];
	
	intrinsicK = cvCreateMat(3,3,CV_64FC1);

/*	for (i=0;i<3;i++)
		for (j=0;j<3;j++)
			CvMat_setdb(intrinsicK,i,j,intK[i][j]);*/
	
}

void FACTORIZATION::Release()
{
	int i,j;
	for (i=0;i<nImg;i++)
	for (j=0;j<nFeature;j++)	
		delete[] data[i][j];
	
	for (i=0;i<nImg;i++)
		delete[] data[i];
	delete[] data;

	for (i=0;i<nImg;i++)
		delete[] CamPos[i];
	delete[] CamPos;

	for (j=0;j<nFeature;j++)
		delete[] p3D[j];
	delete[] p3D;
	delete[] perspMatrix;	
	delete[] imgBe;
	cvReleaseMat(&intrinsicK);
}


void FACTORIZATION::SetCondition()
{
	int i,j,k,l;
	double swapPt[2],img_Width = 1.0; //no scale
	
	for (i=0;i<nImg;i++)
	{
		imgBe[i].centroid[0] =0;
		imgBe[i].centroid[1] =0;
	}

	for (i=0;i<nImg;i++)
	for (j=0;j<nFeature;j++)
	{
		imgBe[i].centroid[0] +=	 data[i][j][0];
		imgBe[i].centroid[1] +=  data[i][j][1];
	}
	for (i=0;i<nImg;i++)
	{
		imgBe[i].centroid[0] /= float(nFeature);
		imgBe[i].centroid[1] /= float(nFeature);	
	}
	//Calculate the average of features in each frame

	for (i=0;i<nImg;i++)
	{	
		CvMat_setdb(imgBe[i].preTrans,0,0,1.0/img_Width);   
		CvMat_setdb(imgBe[i].preTrans,0,1,0.0);
		CvMat_setdb(imgBe[i].preTrans,0,2,-imgBe[i].centroid[0]/img_Width);

		CvMat_setdb(imgBe[i].preTrans,1,0,0.0);   
		CvMat_setdb(imgBe[i].preTrans,1,1,1.0/img_Width);
		CvMat_setdb(imgBe[i].preTrans,1,2,-imgBe[i].centroid[1]/img_Width);

		CvMat_setdb(imgBe[i].preTrans,2,0,0.0);   
		CvMat_setdb(imgBe[i].preTrans,2,1,0.0);
		CvMat_setdb(imgBe[i].preTrans,2,2,1.0);
	}
	//Set Transformation for each frame

	for (i=0;i<nImg;i++)
	{
		for (j=0;j<nFeature;j++)
		{
			matrix_apply_vector2D(swapPt,imgBe[i].preTrans->data.db,data[i][j]);
			data[i][j][0] = swapPt[0];
			data[i][j][1] = swapPt[1];
		}	
	}
	//Normalize all points

	//
		CvMat *S = cvCreateMat(3,3,CV_64FC1);
		CvMat *D = cvCreateMat(3,3,CV_64FC1);
		CvMat *invD = cvCreateMat(3,3,CV_64FC1);
		CvMat *L = cvCreateMat(3,3,CV_64FC1);
		CvMat *U = cvCreateMat(3,3,CV_64FC1);
		CvMat *A = cvCreateMat(nFeature,3,CV_64FC1);
		CvMat *Ap = cvCreateMat(3,nFeature,CV_64FC1);
		CvMat *newP = cvCreateMat(3,nFeature,CV_64FC1);

	for (i=0;i<nImg;i++)
	{
		for (j=0;j<nFeature;j++)
		{
			CvMat_setdb(A,j,0,data[i][j][0]);
			CvMat_setdb(A,j,1,data[i][j][1]);
			CvMat_setdb(A,j,2,1.0);
			CvMat_setdb(Ap,0,j,data[i][j][0]);
			CvMat_setdb(Ap,1,j,data[i][j][1]);
			CvMat_setdb(Ap,2,j,1.0);			
		}
		
		cvMatMul(Ap,A,S);
		CvMat_TLU(L,U,S);

		//set D as diagonal 
		for (j=0;j<3;j++)
			for (k=0;k<3;k++)
				if (j==k)
					CvMat_setdb(D,j,k,CvMat_getdb(U,j,k));
				else
					CvMat_setdb(D,j,k,0.0);
				
		for (j=0;j<3;j++)
		for (k=0;k<3;k++)
			if (j==k) CvMat_setdb(invD,j,k,1/CvMat_getdb(D,j,k));
			else CvMat_setdb(invD,j,k,0.0);

		cvMatMul(invD,U,L);
		cvTranspose(L,U);

		for (j=0;j<3;j++)
			CvMat_setdb(D,j,j,sqrt(CvMat_getdb(D,j,j)/float(nFeature)));

		cvMatMul(U,D,imgBe[i].H);  
		cvInvert(imgBe[i].H,L,CV_LU);
		
		cvMatMul(L,Ap,newP);

		for (j=0;j<nFeature;j++)
		{
			data[i][j][0] = CvMat_getdb(newP,0,j)/CvMat_getdb(newP,2,j);
			data[i][j][1] = CvMat_getdb(newP,1,j)/CvMat_getdb(newP,2,j);
		}
	}

		cvReleaseMat(&S);
		cvReleaseMat(&D);
		cvReleaseMat(&invD);
		cvReleaseMat(&L);
		cvReleaseMat(&U);
		cvReleaseMat(&A);
		cvReleaseMat(&Ap);
		cvReleaseMat(&newP);
	
}

void FACTORIZATION::SaveToSFM(char *fileName)
{
	FILE *stream = fopen(fileName,"w");
	int i,j;
	float pt[3],puv[3];
	
	fprintf(stream,"\n#LABEL 1\n");

	fprintf(stream,"\n\n#POINT %d\n",nFeature);
	for (i=0;i<nFeature;i++)
	    fprintf(stream,"%f %f %f 255 255 255\n",p3D[i][0],p3D[i][1],p3D[i][2]);

	fprintf(stream,"\n\n\n");
	for (i=0;i<nImg;i++)
	{
		fprintf(stream,"#CAMERA %.2d\n",i);
		fprintf(stream,"camera_scale 2.0\n");
		fprintf(stream,"camera_intrinsic\n");
		for (j=0;j<3;j++)
			fprintf(stream,"%f %f %f\n",intK[j][0],intK[j][1],intK[j][2]);
		fprintf(stream,"camera_extrinsic\n");
		for (j=0;j<3;j++)
			fprintf(stream,"%f %f %f %f\n",perspMatrix[i].data[j][0],perspMatrix[i].data[j][1],perspMatrix[i].data[j][2],perspMatrix[i].data[j][3]);
		fprintf(stream,"\n\n");
	}
	
	fclose(stream);

	char fiName[256];
	for (i=0;i<nImg;i++)
	{
		sprintf(fiName,"%s_%.2d.m",fileName,i);
		j=0; while ((j<255)&&(fiName[j]!='\0')) j++; 
		j-=3;
		while (j>0) { if (fiName[j]=='.') fiName[j]='_'; j--;}
		stream = fopen(fiName,"w");
		fprintf(stream,"%%This file shows Original Features & Projective Position.\n");
		fprintf(stream,"\nclear all\n\n");
		
		fprintf(stream,"origF=[");
		for (j=0;j<nFeature-1;j++)
			fprintf(stream,"%f,%f;\n",data[i][j][0],data[i][j][1]);
		fprintf(stream,"%f,%f];\n\n",data[i][j][0],data[i][j][1]);

		fprintf(stream,"projF=[");
		for (j=0;j<nFeature-1;j++)
		{
			pt[0] = DOT3(p3D[j],&(perspMatrix[i].data[0][0])) + perspMatrix[i].data[0][3];
			pt[1] = DOT3(p3D[j],&(perspMatrix[i].data[1][0])) + perspMatrix[i].data[1][3];
			pt[2] = DOT3(p3D[j],&(perspMatrix[i].data[2][0])) + perspMatrix[i].data[2][3];
			puv[0] = DOT3(intK[0],pt);
			puv[1] = DOT3(intK[1],pt);
			puv[2] = DOT3(intK[2],pt);
			fprintf(stream,"%f,%f;\n",puv[0]/puv[2],
				                      puv[1]/puv[2]);
		}
			pt[0] = DOT3(p3D[j],&(perspMatrix[i].data[0][0])) + perspMatrix[i].data[0][3];
			pt[1] = DOT3(p3D[j],&(perspMatrix[i].data[1][0])) + perspMatrix[i].data[1][3];
			pt[2] = DOT3(p3D[j],&(perspMatrix[i].data[2][0])) + perspMatrix[i].data[2][3];
			puv[0] = DOT3(intK[0],pt);
			puv[1] = DOT3(intK[1],pt);
			puv[2] = DOT3(intK[2],pt);
			fprintf(stream,"%f,%f];\n\n",puv[0]/puv[2],
				                      puv[1]/puv[2]);

		fprintf(stream,"plot(origF(:,1),origF(:,2),'r+',projF(:,1),projF(:,2),'bx');\n");
		fprintf(stream,"legend('Original Features','Projective Postions');\n");
		fclose(stream);
	}
}

void FACTORIZATION::ApplyMapping(double *M44)
{
	int i,j;
	double swp[3];
	for (i=0;i<nFeature;i++)
	{
		swp[0] = DOT3(&(M44[0]),p3D[i]) + M44[3];
		swp[1] = DOT3(&(M44[4]),p3D[i]) + M44[7];
		swp[2] = DOT3(&(M44[8]),p3D[i]) + M44[11];
		p3D[i][0] = swp[0];
		p3D[i][1] = swp[1];
		p3D[i][2] = swp[2];
	}
	for (j=0;j<nImg;j++)
	{
		CvMat *cMam = cvCreateMat(4,4,CV_64FC1);
		CvMat *invcMam = cvCreateMat(4,4,CV_64FC1);
		Mult_Matrix(M44,CamPos[j]);
		copy_element(CamPos[j],cMam->data.db,16);
		cvInvert(cMam,invcMam,CV_LU);
		for (int l=0;l<3;l++)
		for (int k=0;k<4;k++)
			perspMatrix[j].data[l][k] = CvMat_getdb(invcMam,l,k);
		cvReleaseMat(&cMam);
		cvReleaseMat(&invcMam);
	}
}

void FACTORIZATION::ApplyScaling(double Sfactor)
{
	int i,j;
	for (i=0;i<nFeature;i++)
	{
		p3D[i][0] *= Sfactor;
		p3D[i][1] *= Sfactor;
		p3D[i][2] *= Sfactor;
	}
	for (j=0;j<nImg;j++)
	{
		CvMat *cMam = cvCreateMat(4,4,CV_64FC1);
		CvMat *invcMam = cvCreateMat(4,4,CV_64FC1);
		CamPos[j][3] *= Sfactor;
		CamPos[j][7] *= Sfactor;
		CamPos[j][11] *= Sfactor;
		copy_element(CamPos[j],cMam->data.db,16);
		cvInvert(cMam,invcMam,CV_LU);
		for (int l=0;l<3;l++)
		for (int k=0;k<4;k++)
			perspMatrix[j].data[l][k] = CvMat_getdb(invcMam,l,k);
		cvReleaseMat(&cMam);
		cvReleaseMat(&invcMam);
	}
}

void FACTORIZATION::ApplyTranslating(double *T)
{
	double M[16];
	Iden_Matrix(M);
	M[3] = T[0];  M[7] = T[1]; M[11] = T[2]; 
	ApplyMapping(M);
}


double FACTORIZATION::Execute(int nIterations)
{
	int i,j,k=0,iterations,local_iteration;
	
	double error_sum = 1e8,tmpVal,prject2D[3];
	CvMat *WS = cvCreateMat(3*nImg,nFeature,CV_64FC1);
	CvMat *U = cvCreateMat(3*nImg,3*nImg,CV_64FC1); ;;
	CvMat *V = cvCreateMat(nFeature,nFeature,CV_64FC1); 
	CvMat *S = cvCreateMat(3*nImg,nFeature,CV_64FC1);  //vector
	CvMat *depth = cvCreateMat(nImg,nFeature,CV_64FC1);
	CvMat *mirror = cvCreateMat(nImg,nFeature,CV_64FC1);
	CvMat *null_matrix;

	CvMat *InvShift = cvCreateMat(3,3,CV_64FC1);
	
	USVLIST *usvList = new USVLIST[nFeature];
	for (i=0;i<nFeature;i++) usvList[i].Create(3*nImg,nImg);
	
	for (i=0;i<nImg*nFeature;i++)
	{
		depth->data.db[i] = 1;
		mirror->data.db[i] = 1;
	}
	
	for (i=0;i<3;i++)
		for (j=0;j<3;j++)
			CvMat_setdb(intrinsicK,i,j,intK[i][j]);
	
//===copy to mirror 
	double ***Mirrordata = new double **[nImg];
	for (i=0;i<nImg;i++)
		Mirrordata[i] = new double* [nFeature];
	
	for (i=0;i<nImg;i++)
	for (j=0;j<nFeature;j++)
	{
		Mirrordata[i][j] = new double[2];
		Mirrordata[i][j][0] = data[i][j][0];
		Mirrordata[i][j][1] = data[i][j][1];
	}
//===	
	SetCondition(); //Determine Centroid and H for each Frame
	
	for (i=0;i<nFeature;i++)
	{
		//Set for each Q
		for (j=0;j<nImg;j++)
		for (k=0;k<nImg;k++)
			if (j==k)
			{
				CvMat_setdb(usvList[i].Q,3*j+0,k,data[j][i][0]);
				CvMat_setdb(usvList[i].Q,3*j+1,k,data[j][i][1]);
				CvMat_setdb(usvList[i].Q,3*j+2,k,1.0);
			}
			else
			{
				CvMat_setdb(usvList[i].Q,3*j+0,k,0.0);
				CvMat_setdb(usvList[i].Q,3*j+1,k,0.0);
				CvMat_setdb(usvList[i].Q,3*j+2,k,0.0);	
			}
		cvSVD(usvList[i].Q,usvList[i].S,usvList[i].U,usvList[i].V,0);
	}

	iterations=0;
	local_iteration = 0;


	while ((error_sum>1e-16)&&(iterations++<nIterations))
	{
		for (i=0;i<nImg;i++)
			for (j=0;j<nFeature;j++)
			{
				CvMat_setdb(WS,3*i  ,j,CvMat_getdb(depth,i,j)*data[i][j][0]);
				CvMat_setdb(WS,3*i+1,j,CvMat_getdb(depth,i,j)*data[i][j][1]);
				CvMat_setdb(WS,3*i+2,j,CvMat_getdb(depth,i,j));
			}
		NormalizeMatrixColumn(WS);

		cvSVD(WS,S,U,V,0);

		CvMat *UT4 =  cvCreateMat(4,3*nImg,CV_64FC1);;
		for (i=0;i<4;i++)
			for (j=0;j<3*nImg;j++)
				CvMat_setdb(UT4,i,j,CvMat_getdb(U,j,i));	

		null_matrix = depth;
		depth = mirror;
		mirror = null_matrix;
		

		for (j=0;j<nFeature;j++)
		{
			CvMat *Rj = cvCreateMat(4,nImg,CV_64FC1);
			CvMat *Aj = cvCreateMat(4,nImg,CV_64FC1);
			cvMatMul(UT4,usvList[j].Q,Rj);  // ( 4 x 3nImg ) * ( 3nImg x nImg )
			cvMatMul(Rj,usvList[j].V,Aj);   // ( 4 x nImg )  * ( nImg  x nImg )

			for (i=0;i<nImg;i++)
				for (k=0;k<4;k++)
					CvMat_setdb(Aj,k,i,CvMat_getdb(Aj,k,i)/CvMat_getdb(usvList[j].S,i,i));

			CvMat *Uaj = cvCreateMat(4,4,CV_64FC1);;
			CvMat *Vaj = cvCreateMat(nImg,nImg,CV_64FC1);;
			CvMat *Saj = cvCreateMat(4,nImg,CV_64FC1);;
			CvMat *hj = cvCreateMat(nImg,1,CV_64FC1);
			CvMat *zj = cvCreateMat(nImg,1,CV_64FC1);
						
			cvSVD(Aj,Saj,Uaj,Vaj,0);
			
			for (i=0;i<nImg;i++)
				CvMat_setdb(hj,i,0, 1.0/(CvMat_getdb(usvList[j].S,i,i))*CvMat_getdb(Vaj,i,0));//
			
			cvMatMul(usvList[j].V,hj,zj);

			for (i=0;i<nImg;i++)
					CvMat_setdb(depth,i,j,CvMat_getdb(zj,i,0)); //

			cvReleaseMat(&Uaj);
			cvReleaseMat(&Saj);
			cvReleaseMat(&Vaj);
			cvReleaseMat(&Rj);
			cvReleaseMat(&Aj);
			cvReleaseMat(&hj);
			cvReleaseMat(&zj);
		}
		
		error_sum = 0;
		for (i=0;i<nImg*nFeature;i++)
			error_sum += power2(depth->data.db[i]-mirror->data.db[i]);

		error_sum = sqrt(error_sum)/float(nImg*nFeature);
		
		cvReleaseMat(&UT4);
		//printf("%d %.18f\n",iterations,error_sum);
	} //End of Iteration for convergence

//==================================================
	for (i=0;i<nImg;i++)
	{
		CvMat *Dhalf = cvCreateMat(3,nFeature,CV_64FC1);
		CvMat *Dhalf2 = cvCreateMat(3,nFeature,CV_64FC1);
		CvMat *TpH = cvCreateMat(3,3,CV_64FC1);
		cvInvert(imgBe[i].preTrans,InvShift,CV_LU);
		copy_element(&(WS->data.db[i*3*nFeature]),Dhalf->data.db,3*nFeature);
		cvMatMul(InvShift,imgBe[i].H,TpH);
		cvMatMul(TpH,Dhalf,Dhalf2);
		copy_element(Dhalf2->data.db,&(WS->data.db[i*3*nFeature]),3*nFeature);
		cvReleaseMat(&TpH);
		cvReleaseMat(&Dhalf);
		cvReleaseMat(&Dhalf2);
	}
	NormalizeMatrixColumn(WS);
	cvSVD(WS,S,U,V,0);
	
//===Determine iterated P===============================
	CvMat *PV = cvCreateMat(4,nFeature,CV_64FC1);
	CvMat *finalP = cvCreateMat(4,nFeature,CV_64FC1);
	CvMat *Mswap = cvCreateMat(3*nImg,4,CV_64FC1);
	CvMat *finalM = cvCreateMat(3*nImg,4,CV_64FC1);
	
	for (j=0;j<4;j++)
		for (k=0;k<nFeature;k++)
			CvMat_setdb(finalP,j,k,CvMat_getdb(V,k,j)); //Assign transpose(V) to finalP (NOTE:Final P should be transpose(V) for first 4 rank)
	
	for (j=0;j<4;j++)
		for (i=0;i<3*nImg;i++)
			CvMat_setdb(Mswap,i,j,CvMat_getdb(U,i,j)*CvMat_getdb(S,j,j)); //Assign M * S to M
	// here, Mswap * finalP == rank4 Matrix
	
	cvInvert(intrinsicK,InvShift,CV_LU);	
	for (i=0;i<nImg;i++)
	{
		CvMat *Dsmhalf = cvCreateMat(3,4,CV_64FC1);
		CvMat *Dsmhalf2 = cvCreateMat(3,4,CV_64FC1);
		copy_element(&(Mswap->data.db[12*i]),Dsmhalf->data.db,12);
		cvMatMul(InvShift,Dsmhalf,Dsmhalf2);
		copy_element(Dsmhalf2->data.db,&(finalM->data.db[12*i]),12);
		cvReleaseMat(&Dsmhalf);
		cvReleaseMat(&Dsmhalf2);
	}
	//So far, we have WS= K * finalM *finalP for rank 4

//============determine Shift of perspective ============
	CvMat *Bq = cvCreateMat(4,1,CV_64FC1);
	CvMat *MNormal = cvCreateMat(2*nImg,4,CV_64FC1);
	CvMat *TNormal = cvCreateMat(2*nImg,1,CV_64FC1);

	for (i=0;i<nImg;i++)
	{
		tmpVal = SumOfList(&(WS->data.db[(3*i+2)*nFeature]),nFeature);
		TNormal->data.db[2*i  ] = SumOfList(&(WS->data.db[(3*i+0)*nFeature]),nFeature)/tmpVal;
		TNormal->data.db[2*i+1] = SumOfList(&(WS->data.db[(3*i+1)*nFeature]),nFeature)/tmpVal;
	}
	
	for (i=0;i<nImg;i++)
	{
		for (j=0;j<4;j++)
		{
			CvMat_setdb(MNormal,2*i  ,j,CvMat_getdb(Mswap,3*i  ,j)/CvMat_getdb(Mswap,3*i+2,j));
			CvMat_setdb(MNormal,2*i+1,j,CvMat_getdb(Mswap,3*i+1,j)/CvMat_getdb(Mswap,3*i+2,j));
		}
	}
	cvSolve(MNormal,TNormal,Bq,CV_SVD);

//============determine Rotation of perspective==========
	for (i=0;i<nImg;i++)
		copy_element(&(finalM->data.db[12*i]),imgBe[i].finM->data.db,12);  //Assign U

	for (i=0;i<nImg;i++)
		copy_element(imgBe[i].finM->data.db,imgBe[i].tmpM->data.db,12);

	CvMat *Bmatrix = cvCreateMat(5*nImg,10,CV_64FC1);
	CvMat *M10 = cvCreateMat(10,1,CV_64FC1);;//gsl_vector_alloc(10);
	CvMat *N10 = cvCreateMat(10,1,CV_64FC1);;//gsl_vector_alloc(10);
	for (i=0;i<nImg;i++)
	{
		SymmetricAssign(imgBe[i].tmpM,M10,0,1);
		//CvMat_setdb_row(Bmatrix,5*i+0,M10);
		copy_element(M10->data.db,&(Bmatrix->data.db[(5*i)*10]),10);
		
		SymmetricAssign(imgBe[i].tmpM,M10,1,2);
		//CvMat_setdb_row(Bmatrix,5*i+1,M10);
		copy_element(M10->data.db,&(Bmatrix->data.db[(5*i+1)*10]),10);

		SymmetricAssign(imgBe[i].tmpM,M10,2,0);
		//CvMat_setdb_row(Bmatrix,5*i+2,M10);
		copy_element(M10->data.db,&(Bmatrix->data.db[(5*i+2)*10]),10);

		SymmetricAssign(imgBe[i].tmpM,N10,1,1);
		SymmetricAssign(imgBe[i].tmpM,M10,0,0);
		
		for (j=0;j<10;j++) M10->data.db[j]=M10->data.db[j]-N10->data.db[j];  //gsl_vector_sub(M10,N10);
		//CvMat_setdb_row(Bmatrix,5*i+3,M10);
		copy_element(M10->data.db,&(Bmatrix->data.db[(5*i+3)*10]),10);
		
		SymmetricAssign(imgBe[i].tmpM,M10,2,2);
		for (j=0;j<10;j++) N10->data.db[j]=N10->data.db[j]-M10->data.db[j];  //gsl_vector_sub(N10,M10);
		//CvMat_setdb_row(Bmatrix,5*i+4,N10);
		copy_element(N10->data.db,&(Bmatrix->data.db[(5*i+4)*10]),10);
	}

	CvMat *SB = cvCreateMat(5*nImg,10,CV_64FC1);
	CvMat *UB = cvCreateMat(5*nImg,5*nImg,CV_64FC1);
	CvMat *VB = cvCreateMat(10,10,CV_64FC1);
	CvMat *Cmatrix = cvCreateMat(4,4,CV_64FC1);
	CvMat *UC = cvCreateMat(4,4,CV_64FC1);
	CvMat *VC = cvCreateMat(4,4,CV_64FC1);
	CvMat *SC = cvCreateMat(4,4,CV_64FC1); //gsl_vector_alloc(4);

	cvSVD(Bmatrix,SB,UB,VB,0);

	CvMat_setdb(Cmatrix,0,0,CvMat_getdb(VB,0,9));
	CvMat_setdb(Cmatrix,0,1,CvMat_getdb(VB,1,9));
	CvMat_setdb(Cmatrix,0,2,CvMat_getdb(VB,2,9));
	CvMat_setdb(Cmatrix,0,3,CvMat_getdb(VB,3,9));
	CvMat_setdb(Cmatrix,1,0,CvMat_getdb(VB,1,9));
	CvMat_setdb(Cmatrix,1,1,CvMat_getdb(VB,4,9));
	CvMat_setdb(Cmatrix,1,2,CvMat_getdb(VB,5,9));
	CvMat_setdb(Cmatrix,1,3,CvMat_getdb(VB,6,9));
	CvMat_setdb(Cmatrix,2,0,CvMat_getdb(VB,2,9));
	CvMat_setdb(Cmatrix,2,1,CvMat_getdb(VB,5,9));
	CvMat_setdb(Cmatrix,2,2,CvMat_getdb(VB,7,9));
	CvMat_setdb(Cmatrix,2,3,CvMat_getdb(VB,8,9));
	CvMat_setdb(Cmatrix,3,0,CvMat_getdb(VB,3,9));
	CvMat_setdb(Cmatrix,3,1,CvMat_getdb(VB,6,9));
	CvMat_setdb(Cmatrix,3,2,CvMat_getdb(VB,8,9));
	CvMat_setdb(Cmatrix,3,3,CvMat_getdb(VB,9,9));
	
	cvSVD(Cmatrix,SC,UC,VC,0);
	
	for (i=0;i<3;i++)
		CvMat_setdb(SC,i,i,sqrt(CvMat_getdb(SC,i,i))); 
	
	for (j=0;j<3;j++)
	for (i=0;i<4;i++)
		CvMat_setdb(UC,i,j,CvMat_getdb(UC,i,j)*CvMat_getdb(SC,j,j));

	for (i=0;i<4;i++)
		CvMat_setdb(UC,i,3,1.0*Bq->data.db[i]);  //UC as QQ
	
	cvInvert(UC,VC,CV_LU);  //VC as inv(QQ)

	copy_element(finalP->data.db,PV->data.db,4*nFeature);//gsl_matrix_memcpy(PV,finalP);
	cvMatMul(VC,PV,finalP);// inv(QQ)*P

	copy_element(finalM->data.db,Mswap->data.db,12*nImg);//gsl_matrix_memcpy(Mswap,finalM);
	cvMatMul(Mswap,UC,finalM);// M*QQ
	//So far, we have  (K*finalM) as project Matrix , finalP as 3D points 
	//Mswap is not useful again

//===Check Mirror direction==================================
	if (CvMat_getdb(finalP,3,0)<0)
	{
		for (i=0;i<3*nImg;i++)   CvMat_setdb(finalM,i,3,-CvMat_getdb(finalM,i,3));
		for (j=0;j<nFeature;j++) CvMat_setdb(finalP,3,j,-CvMat_getdb(finalP,3,j));
	}
	double camU[3],camV[3],camW[3],CamDir[3];
	for (j=0;j<3;j++)
	{
		camU[j] = 1000*CvMat_getdb(finalM,j,0);
		camV[j] = 1000*CvMat_getdb(finalM,j,1);
		camW[j] = 1000*CvMat_getdb(finalM,j,2);
	}
	CDOT(CamDir,camU,camV);
	if (DOT3(CamDir,camW) < 0 )
	{
		for (i=0;i<3*nImg;i++)   CvMat_setdb(finalM,i,1,-CvMat_getdb(finalM,i,1));
		for (j=0;j<nFeature;j++) CvMat_setdb(finalP,1,j,-CvMat_getdb(finalP,1,j));
	}

	for (j=0;j<nFeature;j++) //normalize
	{
		CvMat_setdb(finalP,0,j,CvMat_getdb(finalP,0,j)/CvMat_getdb(finalP,3,j));
		CvMat_setdb(finalP,1,j,CvMat_getdb(finalP,1,j)/CvMat_getdb(finalP,3,j));
		CvMat_setdb(finalP,2,j,CvMat_getdb(finalP,2,j)/CvMat_getdb(finalP,3,j));
		CvMat_setdb(finalP,3,j,1.0);
	}

	for (i=0;i<nImg;i++)
		copy_element(&(finalM->data.db[12*i]),imgBe[i].finM->data.db,12);  //Assign back real_M


//====FINAL STAGE=====
	for (j=0;j<nFeature;j++)
	{
		p3D[j][0] = CvMat_getdb(finalP,0,j);	p3D[j][1] = CvMat_getdb(finalP,1,j);	p3D[j][2] = CvMat_getdb(finalP,2,j);
	}
	for (j=0;j<3;j++) prject2D[j] = 0;
	for (j=0;j<nFeature;j++)
		for (k=0;k<3;k++)
			prject2D[k] += p3D[j][k];
	for (j=0;j<3;j++) prject2D[j] /= float(nFeature);
	for (j=0;j<nFeature;j++)
		for (k=0;k<3;k++)
			p3D[j][k] -= prject2D[k];  // SHIFT Center to original

	
	//store Perspective in output data  // also normalize it
	for (i=0;i<nImg;i++)
	{
		tmpVal =  sqrt (power2(CvMat_getdb(imgBe[i].finM,0,0))+power2(CvMat_getdb(imgBe[i].finM,1,0))+power2(CvMat_getdb(imgBe[i].finM,2,0)) );
			    //+ sqrt (power2(CvMat_getdb(imgBe[i].finM,0,1))+power2(CvMat_getdb(imgBe[i].finM,1,1))+power2(CvMat_getdb(imgBe[i].finM,2,1)) )
				//+ sqrt (power2(CvMat_getdb(imgBe[i].finM,0,2))+power2(CvMat_getdb(imgBe[i].finM,1,2))+power2(CvMat_getdb(imgBe[i].finM,2,2)) );
		CvMat *invCam = cvCreateMat(4,4,CV_64FC1);
		CvMat *DstCam = cvCreateMat(4,4,CV_64FC1);
		
		for (j=0;j<3;j++)
		for (k=0;k<4;k++)
		{
			perspMatrix[i].data[j][k] = CvMat_getdb(imgBe[i].finM,j,k)/tmpVal;
			CvMat_setdb(invCam,j,k,perspMatrix[i].data[j][k]);
		}
		for (k=0;k<3;k++)	
			CvMat_setdb(invCam,3,k,0);
		CvMat_setdb(invCam,3,k,1);
		cvInvert(invCam,DstCam,CV_LU);

		CvMat_setdb(DstCam,0,3,CvMat_getdb(DstCam,0,3)-prject2D[0]);
		CvMat_setdb(DstCam,1,3,CvMat_getdb(DstCam,1,3)-prject2D[1]);
		CvMat_setdb(DstCam,2,3,CvMat_getdb(DstCam,2,3)-prject2D[2]);
		
		cvInvert(DstCam,invCam,CV_LU);

		for (j=0;j<16;j++)
			CamPos[i][j] = DstCam->data.db[j];
		for (j=0;j<3;j++)
		for (k=0;k<4;k++)
			perspMatrix[i].data[j][k] = CvMat_getdb(invCam,j,k);

		cvReleaseMat(&invCam);
		cvReleaseMat(&DstCam);
	}

	cvReleaseMat(&PV);
	cvReleaseMat(&finalP); 
	cvReleaseMat(&Mswap); 
	cvReleaseMat(&finalM); 
	cvReleaseMat(&Cmatrix);
	cvReleaseMat(&UC);
	cvReleaseMat(&VC);
	cvReleaseMat(&UB);
	cvReleaseMat(&SB);
	cvReleaseMat(&VB);
	cvReleaseMat(&SC);
	cvReleaseMat(&M10);
	cvReleaseMat(&N10);

	cvReleaseMat(&Bq);
	cvReleaseMat(&MNormal);
	cvReleaseMat(&TNormal);

	cvReleaseMat(&Bmatrix);
	cvReleaseMat(&InvShift);
	cvReleaseMat(&WS);
	cvReleaseMat(&V);
	cvReleaseMat(&U);
	cvReleaseMat(&depth);
	cvReleaseMat(&mirror);
	cvReleaseMat(&S);	
	delete[] usvList;

	double ***pMirrordata = data;
	data = Mirrordata;
	for (i=0;i<nImg;i++)
	for (j=0;j<nFeature;j++)
		delete[] pMirrordata[i][j];
	for (i=0;i<nImg;i++) delete[] pMirrordata[i];
	delete[] pMirrordata;

	return 1;
}