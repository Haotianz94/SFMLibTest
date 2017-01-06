#include "vgg_X_from_xP_lin.h"
#include "vgg_contreps.h"
#include "cv_util.h"

// Estimation of 3D point from image matches and camera matrices, linear.
// Added by Utsav
CvMat* vgg_X_from_xP_lin(CvMat *u_in, CvMat **P_in, int K, CvMat *imsize_in)
{
	// make temp copies of u_in, P_in and imsize_in so as to not change original matrices
	CvMat *u = cvCloneMat(u_in);
	CvMat **P=new CvMat* [K];
	int j;
	for(j=0;j<K;j++)
	{
		P[j]=cvCreateMat(3,4,CV_64FC1);
		cvCopy(P_in[j],P[j]);	
	}
	CvMat *imsize = cvCloneMat(imsize_in);

	for (int k=0;k<K;k++) 
	{
		// get matrix H
		CvMat *H = cvCreateMat(3, 3, CV_64FC1);
		cvmSet(H, 0, 0, (2 / cvmGet(imsize, 0, k)));
		cvmSet(H, 0, 1, 0);
		cvmSet(H, 0, 2, -1);
		cvmSet(H, 1, 0, 0);
		cvmSet(H, 1, 1, (2 / cvmGet(imsize, 1, k)));
		cvmSet(H, 1, 2, -1);
		cvmSet(H, 2, 0, 0);
		cvmSet(H, 2, 1, 0);
		cvmSet(H, 2, 2, 1);

		// P(:,:,k) = H*P(:,:,k)
		//CvMat *P_new = cvCreateMat(3, 4, CV_64FC1);
		cvMatMul(H, P[k], P[k]);

		// H(1:2,1:2)
		CvMat *tempMatrix = cvCreateMat(2, 2, CV_64FC1);
		CvRect area;area.x = 0;area.y = 0;area.height = 2;area.width = 2;
		cvGetSubRect(H, tempMatrix, area);
		// u(:,k)
		CvMat *u_col_k = cvCreateMat(u->rows, 1, CV_64FC1);
		cvGetCol(u, u_col_k, k);
		// u(:,k) = H(1:2,1:2)*u(:,k)
		cvMatMul(tempMatrix, u_col_k, u_col_k);
		// H(1:2,3)
		CvMat *tempMatrix2 = cvCreateMat(2, 1, CV_64FC1);
		area.x = 2;area.y = 0;area.height = 2;area.width = 1;
		cvGetSubRect(H, tempMatrix2, area);
		// u(:,k) = H(1:2,1:2)*u(:,k) + H(1:2,3)
		cvAdd(u_col_k, tempMatrix2, u_col_k);

		// set u_col_k to u(:,k)
		for (int i=0;i<u->rows;i++) 
		{
			cvmSet(u, i, k, cvmGet(u_col_k, i, 0));
		}

		// remember to clean up
		cvReleaseMat(&H);
		cvReleaseMat(&tempMatrix);
		cvReleaseMat(&u_col_k);
		cvReleaseMat(&tempMatrix2);
	}

	// allocate matrix A
	CvMat *A = cvCreateMat(3*K, 4, CV_64FC1);
	for (int k=0;k<K;k++)
	{
		// u(:,k)
		CvMat *u_col_k = cvCreateMat(u->rows+1, 1, CV_64FC1);
		for (int i=0;i<u->rows;i++) {
			cvmSet(u_col_k, i, 0, cvmGet(u, i, k));
		}
		//[u(:,k);1]
		cvmSet(u_col_k, u_col_k->rows-1, 0, 1);
		// vgg_contreps([u(:,k);1])
		CvMat *contreps = vgg_contreps(u_col_k);
		// vgg_contreps([u(:,k);1])*P(:,:,k)
		CvMat * temp = cvCreateMat(3, 4, CV_64FC1);
		cvMatMul(contreps, P[k], temp);
		for (int row=0;row<temp->rows;row++) {
			for (int col=0;col<A->cols;col++) {
				cvmSet(A, k*temp->rows+row, col, cvmGet(temp, row, col));
			}
		}
		
		// remember to clean up
		cvReleaseMat(&u_col_k);
		cvReleaseMat(&contreps);
		cvReleaseMat(&temp);

	}

	// [dummy,dummy,X] = svd(A,0);
	CvMat *U, *V, *W;
	U = cvCreateMat(A->rows, A->rows, CV_64FC1);
	V = cvCreateMat(A->cols, A->cols, CV_64FC1);
	W = cvCreateMat(A->rows, A->cols, CV_64FC1);

	cvSVD(A, W, U, V);
	// X = X(:,end);
	CvMat *X = cvCreateMat(V->rows, 1, CV_64FC1);

	X=sfmGetCols(V,V->cols-1);
	//X = cvGetCol(V, X, V->cols-1);

	// get orientation right
	// reshape(P(3,:,:),[4 K])
	CvMat *tempMat = cvCreateMat(4, K, CV_64FC1);
	for (int row=0;row<tempMat->rows;row++) {
		for (int col=0;col<tempMat->cols;col++) {
			cvmSet(tempMat, row, col, cvmGet(P[col], 2, row));
		}
	}
	// s = reshape(P(3,:,:),[4 K])'*X;
	CvMat *tempMatTran = cvCreateMat(K, 4, CV_64FC1);
	cvTranspose(tempMat, tempMatTran);
	CvMat *s = cvCreateMat(K, 1, CV_64FC1);
	cvMatMul(tempMatTran, X, s);

	// if any(s<0)
	bool negative = false;
	for (int i=0;i<s->rows;i++) {
		for (int j=0;j<s->cols;j++) {
			if (cvmGet(s, i, j) < 0) {
				negative = true;
			}
		}
	}
	
	if (negative) {
		cvConvertScale(X, X, -1);
	}
	
	// remember to clean up
	cvReleaseMat(&A);
	cvReleaseMat(&U);
	cvReleaseMat(&W);
	cvReleaseMat(&tempMat);
	cvReleaseMat(&tempMatTran);
	cvReleaseMat(&s);
	cvReleaseMat(&u);
	
	cvReleaseMat(&imsize);
	cvReleaseMat(&V);

	cvReleaseMatGrp(P,K);

	return X;
}