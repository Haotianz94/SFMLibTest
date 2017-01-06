//SIFT Header file
#ifndef _TRAJ_H_
#define _TRAJ_H_
#include <stdio.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//--- Structure for single trajectory node
//output data structure
#define GO_m_pNext(A) A = A->m_pNext
struct LinkNode
{
	LinkNode();
	~LinkNode();
	int value;
	struct LinkNode* next;
};
struct Link
{
	int num_node;
	struct LinkNode* head;
	struct LinkNode* tail;
	Link();
	~Link();
	void AddValue(int value);
	int GetValue(int index);
	bool find(int fvalue);
	void Release();
};
struct trajnode
{
	int m_nFrameNo;
	int m_nFeatureNo;
	double m_fx;
	double m_fy;
	struct trajnode* m_pNext;
	//--- ctor & dtor
	trajnode();
	~trajnode();
	//--- member functions
	void setTrajnode(int nFrameNo, int nFeatureNo, double fx, double fy);
	void setNextnode(struct trajnode* ptrajnode);
};

//--- Structure for single trajectory, composed by multiple trajnode
struct traj
{
	int m_nTrajNo;//feature index
	int m_nSFNo;
	int m_nEFNo;
	double RGB[3];//Design for RGB,but should be compatible with greyscale image!

	struct trajnode* m_pTNhead;
	struct trajnode* m_pTNtail;
	struct traj* m_pNext; 
	//--- ctor & dtor
	traj();
	~traj();
	//--- member functions
	void setTraj(int nTrajNo);
	int addTrajnode(int nFrameNo, int nFeatureNo, double fx, double fy);
	int addNode(int nFrameNo,int nFeatureNo,double fx,double fy);//a function added by Zhaoyin
	void setNexttraj(struct traj* ptraj);

	//Function to screen
	traj* GetInterLink(int interSF,int interEF);
	trajnode* GetNode(int index);

	void Release();
};

//--- Structure for trajectory group, composed by multiple traj
struct trajgrp {
	char* pcFNameFmt;//the input file format, like "nt%04d.jpg"
	int m_nTrajNum;//the feature number of the group
	struct traj* m_ptrajHead;
	struct traj* m_ptrajTail;
	//--- ctor & dtor
	int SFNo;
	int EFNo;
	//variable to store the starting frame number and ending frame number;	

	struct trajnode*** pTrajArr;
	//a array storing all the trajnode points for efficiency. Added by Zhaoyin

	int imgW;
	int imgH;
	//variable to store the image width and height;

	trajgrp();
	~trajgrp();
	//--- member functions
	int addTraj();
	
	//Destruction function
	void TrajRelease();

	//Kevin's function
	int addTrajNode(int nPreFrameNo, int nPreFeaNo, double fPreX, double fPreY, int nCurFrameNo, int nCurFeaNo, double fCurX, double fCurY);
	void printTrajList(int nMinLen = 0);
	void drawTrajList(IplImage* pimgBase, int MinLen, int nNumPerCol, int nImgW, int nImgH, int nStartFrameNo);

	//Put the linknode to an array
	void Link2Arr();

	//Get trajIndex th interlink from frame SF to frame EF.Feature Number is a necessary information to provide for efficiency
	traj* GetInterTraj(int trajIndex,int interSF,int interEF,traj* pCur,int m_nTrajNo);

	//output to an .m file
	int outTrajList(char* pcFName, int nMinLen = 0);

	//output for cpp reading
	int outTrajCpp(char* pcFName,int nMinLen=0);
 
	//read cpp data
	int readTrajCpp(char* pcFName);

	//draw the feature index in each image.
	void DrawFeaturesIndex(char* inFmt="FeaturesIndex%04d.jpg", double dblScale=0.3);

	//get the color of each feature,from the image directly
	void GetFeatureColor();
	//get the color of each feature from the feature file
	int GetFeatureColor(char* pcFName);
	
	//output the feature color
	int outFeatureColor(char* pcFName);

	//draw the feature of the specified Image "ImgNo", and save to "fileName"
	void DrawFeaturesOfOneImg(char *fileName,int ImgNo);

	//Compare the feature index wiht other trajgrp: 
	//the table1[i]-th traj of main trajgrp and table1[i]-th of comptrajgrp have the same feature index. 
	//Note you should free momory for table1 & table2 after use.
	void CompareIndexWithOther(struct trajgrp *comtrajgrp,int **table1,int **table2,int *num);

};



#endif // !_TRAJ_H_