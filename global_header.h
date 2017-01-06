#pragma once
#pragma warning (disable: 4996)
#pragma warning(disable:4035) //no return value
#pragma warning(disable:4068) // unknown pragma
#pragma warning(disable:4305) //no return value
#pragma warning(disable:4201) //nonstandard extension used : nameless struct/union
#pragma warning(disable:4267)
#pragma warning(disable:4018) //signed/unsigned mismatch
#pragma warning(disable:4127) //conditional expression is constant
#pragma warning(disable:4146)
#pragma warning(disable:4244) //conversion from 'LONG_PTR' to 'LONG', possible loss of data
#pragma warning(disable:4311) //'type cast' : pointer truncation from 'BYTE *' to 'ULONG'
#pragma warning(disable:4312) //'type cast' : conversion from 'LONG' to 'WNDPROC' of greater size
#pragma warning(disable:4346) //_It::iterator_category' : dependent name is not a type
#pragma warning(disable:4786)
#pragma warning(disable:4541) //'dynamic_cast' used on polymorphic type
#pragma warning(disable:4996) //declared deprecated ?
#pragma warning(disable:4200) //zero-sized array in struct/union
#pragma warning(disable:4800) //forcing value to bool 'true' or 'false' (performance warning)

/*Common header*/
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <string.h>
#include <cstdio>
#include <vector>
#include <bitset>
#include <cmath>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <ctime>
#include <fstream>
#include <omp.h>
#include <windows.h>

//OpenCV header
#include <opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
/*
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/ml/ml.hpp>*/

#define FOR(i,a,b) for (int i(a); i < (b); i++)

#define FOR_REVERSE(i,a,b) for( int i(b-1); i >=(a); i--)


#define REP(i,b) for (int i(0); i < (b); i++)
#define SORT(v) sort((v).begin(),(v).end())
#define ALL(v) ((v).begin(),(v),end())
#define UNIQUE(v) sort((v).begin(),(v).end()),v.erase(unique(v.begin(),v.end()),v.end())
#define CLR(a,b) memset(a,b,sizeof(a))
#define SZ(Z) ((int)(Z).size())
#define PB push_back
#define MP make_pair
//#define EPS 1e-6
#define INFINITY 1e8

#define SET_FONT_GREEN {SetConsoleTextAttribute( GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_GREEN);}

#define SET_FONT_RED {SetConsoleTextAttribute( GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED);}

#define SET_FONT_BLUE {SetConsoleTextAttribute( GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_BLUE);}

#define SET_FONT_YELLOW {SetConsoleTextAttribute (GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY |FOREGROUND_RED | FOREGROUND_GREEN);}

#define SET_FONT_MAGENTA {SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_BLUE);}

#define SET_FONT_CYAN {SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | FOREGROUND_GREEN | FOREGROUND_BLUE);}

#define SET_FONT_WHITE {SetConsoleTextAttribute( GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED);}


//if(img.cols>1000||img.rows>1000)\

#define SHOW_IMG(img){\
	cvNamedWindow(#img,CV_WINDOW_KEEPRATIO);\
	imshow(#img, img); \
	waitKey(); \
}

#define SHOW_IMG_NO_WAIT(img){\
	cvNamedWindow(#img,CV_WINDOW_KEEPRATIO);\
	imshow(#img, img); \
}

#define SHOW_IMG_WAIT(img, time){\
	cvNamedWindow(#img,CV_WINDOW_KEEPRATIO);\
	imshow(#img, img); \
	waitKey(time); \
}



#define SET_CONSOLE(title){\
	system("mode con: cols=100");\
	SetConsoleTitle(L#title);\
}

#define SEG_LINE {\
	SET_FONT_CYAN;\
	cout<<"--------------------------------------------------------------------------"<<endl;\
	SET_FONT_WHITE;\
}


//const double PI = acos(-1.0);
/*
#define PI 3.141592653
#define SQRT_2 	1.41421356
#define SQRT_SQRT_2 1.189207		*/		 



#include "opencv2/core/version.hpp"
/*#include "opencv2/features2d.hpp"*/
/*#include "opencv2/xfeatures2d.hpp"*/
//#include "opencv2/stitching/detail/seam_finders.hpp"
//#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/gpu.hpp>
//#include <opencv2/gpu/gpu.hpp>



#define CV_VERSION_ID  CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
#define cvLIB(name) "opencv_" name CV_VERSION_ID "d"
#else
#define cvLIB(name) "opencv_" name CV_VERSION_ID
#endif


// 
#pragma comment(lib, cvLIB("photo"))
#pragma comment(lib, cvLIB("core"))
#pragma comment(lib, cvLIB("features2d"))
//#pragma comment(lib, cvLIB("xfeatures2d"))
#pragma comment(lib, cvLIB("highgui"))
#pragma comment(lib, cvLIB("imgproc"))
#pragma comment(lib, cvLIB("flann"))
#pragma comment(lib, cvLIB("nonfree"))
#pragma comment(lib, cvLIB("objdetect"))
#pragma comment(lib, cvLIB("ml"))
//#pragma comment(lib, cvLIB("stitching"))
//#pragma comment(lib, cvLIB("legacy"))
#pragma comment(lib, cvLIB("video"))
//#pragma comment(lib, cvLIB("videoio"))
#pragma comment(lib, cvLIB("calib3d"))
#pragma comment(lib, cvLIB("gpu"))

//#pragma comment(lib, cvLIB("imgcodecs"))

//#pragma comment(lib, cvLIB("ts"))
//#pragma comment(lib, cvLIB("world"))

using namespace std;
using namespace cv;

/*
typedef long long LL;
typedef vector<int> VI;
typedef vector<string> VS;
typedef pair<int,int> PII;
typedef pair<double,double> PDD;*/

#define FOR_PIXELS(y,x,IMG) for(int y=0;y<(IMG).rows;++y)for(int x=0;x<(IMG).cols;++x)
#define INSIDE_IMG(y,x,IMG) ((y)>=0 && (y)<IMG.rows && (x)>=0 && (x)<IMG.cols)
#define RC(IMG) (IMG).rows, (IMG).cols
#define CR(IMG) (IMG).cols, (IMG).rows
#define CV_WM_ERROR 1
#define WM_ERROR( expr ) cv::error( cv::Exception(CV_WM_ERROR, expr, "", __FILE__, __LINE__) )



#define miaoLog(fout, SENTENCE) {\
	fout<<SENTENCE; \
	cout<< SENTENCE;\
	fout.flush();\
}


#define min_Miao( a , b ) (a) < (b) ? (a): (b)


#define GLOG_NO_ABBREVIATED_SEVERITIES
//Ceres Solver Library
#include "ceres/ceres.h"
#include "glog/logging.h"
#pragma comment(lib, "ceres.lib")
#pragma comment(lib, "libglog_static.lib")

//#define WM_DEBUG