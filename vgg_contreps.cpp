#include "vgg_contreps.h"

// Contraction with epsilon tensor
// added by Utsav
CvMat* vgg_contreps(CvMat *X)
{
	// we assume that X is 3*1 vector
	  
	CvMat *Y = cvCreateMat(3, 3, CV_64FC1);
	// get the steps for both matrices
	int iRowStepY = Y->step / sizeof( double );
	int iRowStepX = X->step / sizeof( double );

	// fill values for Y
	Y->data.db[0*iRowStepY+0] = 0;
	Y->data.db[0*iRowStepY+1] = X->data.db[2*iRowStepX+0];
	Y->data.db[0*iRowStepY+2] = -X->data.db[1*iRowStepX+0];
	Y->data.db[1*iRowStepY+0] = -X->data.db[2*iRowStepX+0];
	Y->data.db[1*iRowStepY+1] = 0;
	Y->data.db[1*iRowStepY+2] = X->data.db[0*iRowStepX+0];
	Y->data.db[2*iRowStepY+0] = X->data.db[1*iRowStepX+0];
	Y->data.db[2*iRowStepY+1] = -X->data.db[0*iRowStepX+0];
	Y->data.db[2*iRowStepY+2] = 0;

	return Y;
}