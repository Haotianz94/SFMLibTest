//SIFT Header file
#ifndef _MATCHSEQ_H
#define _MATCHSEQ_H


/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49// 0.3 
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "traj.h"


int set_featureno(int nImgNo, int nFeaNum, struct feature* pfeature);
IplImage* DrawSubImg( IplImage* pimgBase, IplImage* pimgSub, int nLocX, int nLocY );
IplImage* DrawSubImgFromSubImg(IplImage* pimgDst, IplImage* pimgSrc, int nImgWidth, int nImgHeight, int nLocXDst, int nLocYDst, int nLocXSrc, int nLocYSrc);

#endif