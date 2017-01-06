/*
Detects SIFT features across image sequence
by Yao-Jen Chang, Advanced Multimedia Processing Lab, ECE, Carnegie Mellon University.

based on SIFT codes by Rob Hess <hess@eecs.oregonstate.edu> version 1.1.1-20070913
*/

#include "matchseq.h"


int set_featureno(int nImgNo, int nFeaNum, struct feature* pfeature)
{
	int i;
	for (i = 0; i < nFeaNum; i++) {
		pfeature[i].m_nImgNo = nImgNo;
		pfeature[i].m_nFeaNo = i;
	}
	return 1;
}

IplImage* DrawSubImg( IplImage* pimgBase, IplImage* pimgSub, int nLocX, int nLocY )
{
	cvSetImageROI( pimgBase, cvRect( nLocX, nLocY, pimgSub->width, pimgSub->height ) );
	cvSetZero(pimgBase);
	cvAdd( pimgSub, pimgBase, pimgBase, NULL );
	cvResetImageROI( pimgBase );
	return pimgBase;
}

IplImage* DrawSubImgFromSubImg(IplImage* pimgDst, IplImage* pimgSrc, int nImgWidth, int nImgHeight, int nLocXDst, int nLocYDst, int nLocXSrc, int nLocYSrc)
{
	cvSetImageROI( pimgSrc, cvRect( nLocXSrc, nLocYSrc, nImgWidth, nImgHeight ) );
	cvSetImageROI( pimgDst, cvRect( nLocXDst, nLocYDst, nImgWidth, nImgHeight ) );
	cvSetZero(pimgDst);
	cvAdd( pimgDst, pimgSrc, pimgDst, NULL );
	cvResetImageROI( pimgSrc );
	cvResetImageROI( pimgDst );
	return pimgDst;
}
