#ifndef VGG_X_FROM_XP_NONLIN
#define VGG_X_FROM_XP_NONLIN

#include <cv.h>
#include <stdio.h>
#include "vgg_X_from_xP_lin.h"
#include "vgg_contreps.h"
#include "cv_util.h"

/* Conversion from quaternion and translation to euler angles and camera center */
CvMat* q2rot(CvMat* q_nion);

/* Evaluate the residual */
int resid(CvMat* Y, CvMat* u, CvMat** Q, int num_Q_mats, CvMat* e, CvMat* J);

/* Concatenate two matrices */
/* dim: 1 = Vertical concatenate, 2 = Horizontal concatenate */
CvMat* cat(CvMat* matA, CvMat* matB, int dim);

/* Find the row-wise or column wise mean of the matrix */
/* mode: 1 = column-wise, 2 = row_wise */
CvMat* mean(CvMat* mat, int mode);

/* Find non-linear estimate of 3D points */
CvMat* vgg_X_from_xP_nonlin(CvMat* u, CvMat** P, CvMat* imsize, int K);
//CvMat* vgg_X_from_xP_nonlin(CvMat u, CvMat* P, CvMat imsize, int K)

#define INF 0x7FFFFFFF
#define EPS 0.00000001 /* 2.2204e-016 */

/* Find linear estimate of 3D points */
//CvMat* vgg_X_from_xP_lin(CvMat *u, CvMat **P, int K, CvMat *imsize);

/* Contraction with epsilon tensor */
//CvMat* vgg_contreps(CvMat *X);

#endif