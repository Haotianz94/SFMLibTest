// Warp.cpp: implementation of the CWarp class.
//
//////////////////////////////////////////////////////////////////////

#include "Warp.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CWarp::CWarp()
{
}

CWarp::~CWarp()
{	
}

CMLS::CMLS(std::vector<CvPoint2D32f> CtrPs,std::vector<CvPoint2D32f> CtrQs)
{
	m_CtrPs=CtrPs;
	m_CtrQs=CtrQs;
}

CvPoint2D32f CMLS::Warping(CvPoint2D32f point)
{
		int nSize=m_CtrPs.size();
		//compute weight
		std::vector<double> Weight;
		for (int i1=0;i1<nSize;i1++)
		{
			double dtemp=( pow(	static_cast<double>(point.x-m_CtrPs[i1].x),  2 )+
										   pow(	static_cast<double>(point.y-m_CtrPs[i1].y),	2)		);

			if(dtemp==0.0)
			{
				dtemp=dtemp+0.0000001;
			}
			
			double temp=1/dtemp;

			Weight.push_back(temp);
		}
		//compute pStar,QStar
		double dSumW=0;
		double dSumPx=0,dSumPy=0;
		double dSumQx=0,dSumQy=0;
		for (int i2=0;i2<nSize;i2++)
		{
			dSumW=dSumW+Weight[i2];
			dSumPx=dSumPx+Weight[i2]*(double)m_CtrPs[i2].x;
			dSumPy=dSumPy+Weight[i2]*(double)m_CtrPs[i2].y;
			dSumQx=dSumQx+Weight[i2]*(double)m_CtrQs[i2].x;
			dSumQy=dSumQy+Weight[i2]*(double)m_CtrQs[i2].y;
		}
		CvPoint2D32f PStar,QStar;
		PStar.x=(dSumPx/dSumW);
		PStar.y=(dSumPy/dSumW);
		QStar.x=(dSumQx/dSumW);
		QStar.y=(dSumQy/dSumW);

		std::vector<CvPoint2D32f> PHat,QHat;
		for (int i3=0;i3<nSize;i3++)
		{
			CvPoint2D32f tempP;
			tempP.x=m_CtrPs[i3].x-PStar.x;
			tempP.y=m_CtrPs[i3].y-PStar.y;
			PHat.push_back(tempP);
			CvPoint2D32f tempQ;
			tempQ.x=m_CtrQs[i3].x-QStar.x;
			tempQ.y=m_CtrQs[i3].y-QStar.y;
			QHat.push_back(tempQ);
		}
		//compute the inverse Matrix
		matrix2 Mat1={0,0,0,0};
		for (int i4=0;i4<nSize;i4++)
		{
			Mat1.c11=Mat1.c11+Weight[i4]* pow(	static_cast<double>(PHat[i4].x), 2 );
			Mat1.c12=Mat1.c12+Weight[i4]*(double)(PHat[i4].x*PHat[i4].y);
			Mat1.c21=Mat1.c12;
			Mat1.c22=Mat1.c22+Weight[i4]* pow( static_cast<double>(PHat[i4].y), 2 );
		}

		double tempM=Mat1.c11*Mat1.c22-Mat1.c12*Mat1.c21;
		matrix2 MatInver;
		MatInver.c11=Mat1.c22/tempM;
		MatInver.c12=-Mat1.c12/tempM;
		MatInver.c21=-Mat1.c21/tempM;
		MatInver.c22=Mat1.c11/tempM;

		//compute the scale Aj
		std::vector<double> dScale;
		for (int i5=0;i5<nSize;i5++)
		{
			double tempS;
			tempS=((double)(point.x-PStar.x)*MatInver.c11+(double)(point.y-PStar.y)*MatInver.c21)*Weight[i5]*(double)PHat[i5].x
				+((double)(point.x-PStar.x)*MatInver.c12+(double)(point.y-PStar.y)*MatInver.c22)*Weight[i5]*(double)PHat[i5].y;
			dScale.push_back(tempS);
		}
		//compute the New Point
		CvPoint2D32f NewPoint=cvPoint2D32f(0,0);
		for (int i6=0; i6<nSize; i6++)
		{
			NewPoint.x=NewPoint.x+(dScale[i6]*QHat[i6].x);
			NewPoint.y=NewPoint.y+(dScale[i6]*QHat[i6].y);
		}
		NewPoint.x=NewPoint.x+QStar.x;
		NewPoint.y=NewPoint.y+QStar.y;
		return NewPoint;	
}