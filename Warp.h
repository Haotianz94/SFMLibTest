// Warp.h: interface for the CWarp class.
//
//////////////////////////////////////////////////////////////////////
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
struct matrix2 
{
	double c11,c12,c21,c22;
};

class CWarp  
{
protected:
	std::vector<CvPoint2D32f> m_CtrPs,m_CtrQs;
//	CPoint m_Point;
//	Matrix2  m_Ma;
public:
	CWarp();
	virtual CvPoint2D32f Warping(CvPoint2D32f point)=0;	
	virtual ~CWarp();
};

class CMLS: public CWarp
{
public:
	CMLS();
	CMLS(std::vector<CvPoint2D32f> CtrPs,std::vector<CvPoint2D32f> CtrQs);
	virtual CvPoint2D32f Warping(CvPoint2D32f point);	
};