
#include "traj.h"
#include "SFMLib.h"
#include <stdio.h>
//---
// trajnode
//--- ctor & dtor
LinkNode::LinkNode()
{
	value=-1;
	next=NULL;
}
Link::Link()
{
	num_node=0;
	head=NULL;
	tail=NULL;
}
LinkNode::~LinkNode()
{
}
void Link::AddValue(int value)
{
	LinkNode* pnode=new LinkNode;	
	pnode->value=value;
	if(head==NULL)
	{
		head=pnode;		
	}
	else
	{
		tail->next=pnode;
	}
	tail=pnode;
	num_node++;
}
bool Link::find(int fvalue)
{
	LinkNode* pnode=head;
	if(num_node==0)
		return 0;
	while(pnode!=NULL)
	{
		if(pnode->value==fvalue)
			return 1;
		pnode=pnode->next;
	}
	return 0;
}
void Link::Release()
{
	LinkNode* pnode=head;
	while(pnode!=NULL)
	{
		tail=pnode;
		pnode=pnode->next;
		delete tail;		
	}
	delete this;
}
Link::~Link()
{
}
trajnode::trajnode() 
{
	m_nFrameNo = m_nFeatureNo = -1;
	m_fx = m_fy = -1.0;
	m_pNext = NULL;
}

int Link::GetValue(int index)//index start from 0
{
	int i;
	LinkNode* pnode=head;

	for(i=0;i<index;i++)
	{
		pnode=pnode->next;
	}
	return pnode->value;
}

trajnode::~trajnode() 
{
	//printf("\tTN(%d,%d) has been deleted!\n", m_nFrameNo, m_nFeatureNo);
	//delete m_pNext;
}

//--- member functions
inline void trajnode::setTrajnode(int nFrameNo, int nFeatureNo, double fx, double fy) 
{
	m_nFrameNo = nFrameNo;
	m_nFeatureNo = nFeatureNo;
	m_fx = fx; m_fy = fy;
}

inline void trajnode::setNextnode(struct trajnode* ptrajnode) 
{
	m_pNext = ptrajnode;
}


//---
// traj
//--- ctor & dtor
traj::traj() 
{
	m_nTrajNo = -1;
	m_nSFNo = -1;
	m_nEFNo = -1;
	m_pTNhead = NULL;
	m_pTNtail = NULL;
	m_pNext = NULL;
	int i;
	for(i=0;i<3;i++)
	{
		RGB[i]=0;
	}

}
traj::~traj() 
{
	//printf("Traj(%d,%d,%d) has been deleted!\n", m_nTrajNo, m_nSFNo, m_nEFNo);
	//delete m_pNext;
	//delete m_pTNhead;
}
void traj::Release()
{
	trajnode* pnode=m_pTNhead;
	while(pnode!=NULL)
	{
		m_pTNtail=pnode;
		pnode=pnode->m_pNext;
		delete m_pTNtail;
	}
	delete this;
}
//--- member function
inline void traj::setTraj(int nTrajNo) 
{
	m_nTrajNo = nTrajNo;
}

inline int traj::addTrajnode(int nFrameNo, int nFeatureNo, double fx, double fy) 
{
	trajnode* ptrajnode = new trajnode;
	ptrajnode->setTrajnode(nFrameNo, nFeatureNo, fx, fy);
	if (m_pTNtail != NULL) m_pTNtail->setNextnode(ptrajnode);
	m_pTNtail = ptrajnode;
	if (m_pTNhead == NULL) {
		m_pTNhead = m_pTNtail;
		m_nSFNo = m_nEFNo = nFrameNo;
	} else {
		if (nFrameNo == m_nEFNo+1)	{
			m_nEFNo = nFrameNo;
		} else {
			printf("Error in adding new trajnode with Frame#%d to Frame#%d\n", nFrameNo, m_nEFNo);  
			return 0;
		}
	}
	return 1;
}
int traj::addNode(int nFrameNo,int nFeatureNo,double fx,double fy)
{
	//this is a function added by Zhaoyin 
	//to setup the points
	if( m_pTNhead==NULL)
	{
		m_pTNhead=new trajnode;
		m_pTNtail=m_pTNhead;
	}
	else
	{
		m_pTNtail->m_pNext=new trajnode;
		m_pTNtail=m_pTNtail->m_pNext;
	}
	m_pTNtail->m_fx=fx;
	m_pTNtail->m_fy=fy;
	m_pTNtail->m_nFeatureNo=nFeatureNo;
	m_pTNtail->m_nFrameNo=nFrameNo;
	
	return 1;	
}
//Function for screening, get a interception link node
traj* traj::GetInterLink(int interSF,int interEF)
{
	traj* ptraj=new traj;
	trajnode* trajnodeCur;
	int i,nFrameNo,nFeatureNo;
	double fx,fy;

	//new starting and ending frames
	ptraj->m_nSFNo=interSF;
	ptraj->m_nEFNo=interEF;
	ptraj->m_nTrajNo=m_nTrajNo;
	
	for(i=interSF-m_nSFNo,trajnodeCur=GetNode(i);i<=interEF-m_nSFNo;i++)
	{		
		nFrameNo=trajnodeCur->m_nFrameNo;
		nFeatureNo=trajnodeCur->m_nFeatureNo;
		fx=trajnodeCur->m_fx;
		fy=trajnodeCur->m_fy;

		//add a node to the traj;
		ptraj->addNode(nFrameNo,nFeatureNo,fx,fy);
		if(trajnodeCur->m_pNext!=NULL)
			trajnodeCur=trajnodeCur->m_pNext;
	}
	
	return ptraj;
}
//Function for screening, get a node by index;
trajnode* traj::GetNode(int index)//This function changed later. Might need debug!!!
{//index from 0 now
	
	trajnode* ptrajnode;
	int i;
	ptrajnode= m_pTNhead;

	for(i=0;i<index;i++)
	{		
		ptrajnode= ptrajnode->m_pNext;//a fatal bug
	}
	return ptrajnode;
}
inline void traj::setNexttraj(struct traj* ptraj) {
	m_pNext = ptraj;
}

//---
// trajgrp
//--- ctor & dtor
trajgrp::trajgrp() 
{
	m_nTrajNum = 0;
	m_ptrajHead = m_ptrajTail = NULL;

	pTrajArr=NULL;//trajnode address arrays

}
void trajgrp::Link2Arr()
{
	if(m_ptrajHead==NULL)
		return;	

	int i,j;
	pTrajArr=new trajnode**[m_nTrajNum];
	traj* pCur;
	pCur=this->m_ptrajHead;
	int maxLength,tempLength;
	maxLength=pCur->m_nEFNo - pCur->m_nSFNo + 1;
	while(pCur!=NULL)
	{
		tempLength=pCur->m_nEFNo - pCur->m_nSFNo + 1;
		if(tempLength>maxLength)
			maxLength=tempLength;
		pCur=pCur->m_pNext;
	}
	for(i=0;i<m_nTrajNum;i++)
		pTrajArr[i]=new trajnode*[maxLength];	
	//Initial the struct array pointers to each trajnode;
	for(i=0;i<m_nTrajNum;i++)
		for(j=0;j<maxLength;j++)
			pTrajArr[i][j]=NULL;//initialize all the pointers null.
	

	i=0;j=0;
	traj* ptraj=m_ptrajHead;
	trajnode* ptrajnode=ptraj->m_pTNhead;
	while(ptraj!=NULL)
	{	
		//j=ptraj->m_nSFNo;
		j=0;
		ptrajnode=ptraj->m_pTNhead;
		while(ptrajnode!=NULL)	
		{
			pTrajArr[i][j]=ptrajnode;
			ptrajnode=ptrajnode->m_pNext;
			j++;
		}
		ptraj=ptraj->m_pNext;
		i++;
	}
}
traj* trajgrp::GetInterTraj(int trajIndex,int interSF,int interEF,traj* pCur,int m_nTrajNo)
{
	traj* ptraj=new traj;
	int j,nFrameNo,nFeatureNo;
	double fx,fy;
	ptraj->m_nSFNo=interSF;
	ptraj->m_nEFNo=interEF;
	ptraj->m_nTrajNo=m_nTrajNo;

	int traj_SFNo=pCur->m_nSFNo;
	int traj_EFNo=pCur->m_nEFNo;
	
	trajnode* trajnodeCur;
	for(j=interSF-traj_SFNo;j<=interEF-traj_SFNo;j++)
	{
		trajnodeCur=pTrajArr[trajIndex][j];
		nFrameNo=trajnodeCur->m_nFrameNo;
		nFeatureNo=trajnodeCur->m_nFeatureNo;
		fx=trajnodeCur->m_fx;
		fy=trajnodeCur->m_fy;

		//add a node to the traj;
		ptraj->addNode(nFrameNo,nFeatureNo,fx,fy);
		trajnodeCur=trajnodeCur->m_pNext;
	}
	
	return ptraj;
	
}
void trajgrp::TrajRelease()
{
	if(this==NULL)
		return;
	if(this->m_ptrajTail!=NULL)
	{
		traj* pCur;
		trajnode* nodeCur;
		pCur=m_ptrajHead;
		for(pCur=m_ptrajHead;pCur!=NULL;pCur=m_ptrajHead)
		//while(pCur!=NULL)
		{
			//delete each node
			for(nodeCur=pCur->m_pTNhead;nodeCur!=NULL;nodeCur=pCur->m_pTNhead)
			{			
				pCur->m_pTNhead=pCur->m_pTNhead->m_pNext;
				//printf("x:%f\n,y:%f\n",nodeCur->m_fx,nodeCur->m_fy);
				
				delete nodeCur;
			}
			//delete each traj
			//printf("trajno %d\n",pCur->m_nTrajNo);
			m_ptrajHead= m_ptrajHead->m_pNext ;
			
			delete pCur;
		}
	}
	if(pTrajArr==NULL)
	{
		delete this;
		return;
	}
	if( (*pTrajArr[0])!=NULL )
	{
		int i;
		for(i=0;i<m_nTrajNum;i++)
		{
			if(pTrajArr[i]!=NULL)
				delete pTrajArr[i];
		}
		delete pTrajArr;
	}
	delete this;
}
trajgrp::~trajgrp() 
{
	delete m_ptrajHead;
}

//--- member function
int trajgrp::addTraj() 
{
	traj* ptraj = new traj;
	ptraj->setTraj(m_nTrajNum++);
	if (m_ptrajTail != NULL) m_ptrajTail->setNexttraj(ptraj);
	m_ptrajTail = ptraj;
	if (m_ptrajHead == NULL) m_ptrajHead = m_ptrajTail;
	return 1;
}

int trajgrp::addTrajNode(int nPreFrameNo, int nPreFeaNo, double fPreX, double fPreY, int nCurFrameNo, int nCurFeaNo, double fCurX, double fCurY)
{
	traj* ptraj = m_ptrajHead,* ptrajTarget;
	trajnode* ptn;
	ptrajTarget = NULL;
	//--- Search First
	while(ptraj != NULL) {
		ptn = ptraj->m_pTNtail;
		if (ptn->m_nFrameNo == nPreFrameNo) {
			if (ptn->m_nFeatureNo == nPreFeaNo) {
				//printf("\tFound Node#(%d, %d) @ (%.3lf, %.3lf)\n", ptn->m_nFrameNo, ptn->m_nFeatureNo, ptn->m_fx, ptn->m_fy); 
				ptrajTarget = ptraj;
				break;
			}
		}
		ptraj = ptraj->m_pNext;
	}
	if (ptrajTarget == NULL) {
		addTraj();
		m_ptrajTail->addTrajnode(nPreFrameNo, nPreFeaNo, fPreX, fPreY);
		m_ptrajTail->addTrajnode(nCurFrameNo, nCurFeaNo, fCurX, fCurY);
	} else {
		ptrajTarget->addTrajnode(nCurFrameNo, nCurFeaNo, fCurX, fCurY);
	}
	return 1;
}

void trajgrp::printTrajList(int nMinLen /* = 0 */)
{
	traj* ptraj = m_ptrajHead;
	trajnode* ptrajnode;
	int nCnt = 0;
	
	while(ptraj != NULL) {
		if (ptraj->m_nEFNo - ptraj->m_nSFNo + 1 >= nMinLen) {
			printf("Traj# %d: (%d - %d)\n", ptraj->m_nTrajNo, ptraj->m_nSFNo, ptraj->m_nEFNo);
			ptrajnode = ptraj->m_pTNhead;
			while (ptrajnode != NULL) {
				printf("\tNode#(%d, %d) @ (%.3lf, %.3lf)\n", ptrajnode->m_nFrameNo, ptrajnode->m_nFeatureNo, ptrajnode->m_fx, ptrajnode->m_fy); 
				ptrajnode = ptrajnode->m_pNext;
			}
			nCnt++;
		}
		ptraj = ptraj->m_pNext;
	}
	printf("Total: %d qualified trajectories.\n", nCnt);
}

int trajgrp::outTrajList(char* pcFName, int nMinLen /* = 0 */)
{
	FILE* fpOut;
	if ((fpOut = fopen(pcFName, "wt")) == NULL) {
		printf("Unable to write trajlist file %s.\n", pcFName);
		return 0;
	}

	traj* ptraj = m_ptrajHead;
	trajnode* ptrajnode;
	int nCnt = 1;
	ptraj = m_ptrajHead;
	while(ptraj != NULL) {
		if (ptraj->m_nEFNo - ptraj->m_nSFNo + 1 >= nMinLen) {
			fprintf(fpOut, "pnSF(%d)=%d; pnEF(%d)=%d;\n", nCnt, ptraj->m_nSFNo+1, nCnt, ptraj->m_nEFNo + 1);
			fprintf(fpOut, "pts{%d} = [\n", nCnt);
			ptrajnode = ptraj->m_pTNhead;
			while (ptrajnode != NULL) {
				fprintf(fpOut, " %7.3lf", ptrajnode->m_fx); 
				ptrajnode = ptrajnode->m_pNext;
			}
			fprintf(fpOut, "\n");
			ptrajnode = ptraj->m_pTNhead;
			while (ptrajnode != NULL) {
				fprintf(fpOut, " %7.3lf", ptrajnode->m_fy); 
				ptrajnode = ptrajnode->m_pNext;
			}
			fprintf(fpOut, "\n");
			ptrajnode = ptraj->m_pTNhead;
			while (ptrajnode != NULL) {
				fprintf(fpOut, " %7.3lf", 1.0); 
				ptrajnode = ptrajnode->m_pNext;
			}
			fprintf(fpOut, "];\n");
			nCnt++;
		}
		ptraj = ptraj->m_pNext;
	}
	fclose(fpOut);
	return 1;
}
int trajgrp::outTrajCpp(char* pcFName,int nMinLen)
{
	FILE* fpOut;
	if ((fpOut = fopen(pcFName, "wt")) == NULL) {
		printf("Unable to write trajlist file %s.\n", pcFName);
		return 0;
	}

	traj* ptraj = m_ptrajHead;
	trajnode* ptrajnode;
	ptraj = m_ptrajHead;

	fprintf(fpOut,"START_FEATURES\n");
	fprintf(fpOut,"FileName=%s SF=%d EF=%d\n",pcFNameFmt,SFNo,EFNo);
	fprintf(fpOut,"Width=%d Height=%d\n",imgW,imgH);
	while(ptraj != NULL) 
	{
		if (ptraj->m_nEFNo - ptraj->m_nSFNo + 1 >= nMinLen) 
		{
			fprintf(fpOut, "TrajNo=%d pnSF=%d pnEF=%d\n", ptraj->m_nTrajNo, ptraj->m_nSFNo, ptraj->m_nEFNo);

			ptrajnode = ptraj->m_pTNhead;
			while (ptrajnode != NULL) {
				fprintf(fpOut, "x=%7.3lf,y=%7.3lf\n", ptrajnode->m_fx,ptrajnode->m_fy); 
				ptrajnode = ptrajnode->m_pNext;
			}
		}
		ptraj = ptraj->m_pNext;
	}
	//fprintf(fpOut,"END_FEATURES\n");
	fclose(fpOut);
	return 1;

}
int trajgrp::readTrajCpp(char* pcFName)
{
	
	FILE* fpIn;
	if((fpIn=fopen(pcFName,"r"))==NULL)
	{
		printf("Unable to read the trajlist file %s.\n",pcFName);
		return 0;
	}
	char strCmd[20];
	fscanf(fpIn,"%s\n",strCmd);
	if(strcmp(strCmd,"START_FEATURES")!=0)
	{
		printf("not a feature file");
		return 0;
	}

	traj* ptraj;
	//trajnode* ptrajnode;
	int n_SFNo,n_EFNo;
	int trajSF,trajEF;
	int TrajNo;
	int i;
	double n_fx,n_fy;
	
	m_nTrajNum=0;
	pcFNameFmt=new char[50];
	fscanf(fpIn,"FileName=%s SF=%d EF=%d\n",pcFNameFmt,&SFNo,&EFNo);
	//fprintf(fpOut,"FileName=%s SF=%d EF=%d\n",pcFNameFmt,SFNo,EFNo);
	fscanf(fpIn,"Width=%d Height=%d\n",&imgW,&imgH);
	while(fscanf(fpIn,"TrajNo=%d pnSF=%d pnEF=%d\n",&TrajNo,&trajSF,&trajEF)!=EOF)
	{	
		m_nTrajNum++;
		if(m_ptrajHead==NULL)
		{
			m_ptrajHead=new traj;
			ptraj=m_ptrajHead;
			n_SFNo=trajSF;
		}
		else
		{
			ptraj->m_pNext=new traj;
			ptraj=ptraj->m_pNext;
		}

		ptraj->m_nSFNo=trajSF;
		ptraj->m_nEFNo=trajEF;
		ptraj->m_nTrajNo=TrajNo;
		for(i=1;i<=trajEF-trajSF+1;i++)
		{
			fscanf(fpIn,"x=%lf,y=%lf\n",&n_fx,&n_fy);
			ptraj->addNode(trajSF+i-1,TrajNo,n_fx,n_fy);
		}

	}
	n_EFNo=trajEF;

	m_ptrajTail=ptraj;
	
	//change linknode to array
	Link2Arr();
	fclose(fpIn);
	return 1;

}
void trajgrp::drawTrajList(IplImage* pimgBase, int nMinLen, int nNumPerCol, int nImgW, int nImgH, int nStartFrameNo)
{
	traj* ptraj = m_ptrajHead;
	trajnode* ptrajnode;
	int nCnt = 0;
	int nOffsetX, nOffsetY;
	CvPoint pt1, pt2;
	CvScalar cvClr;
	int pnclrr[] = {0,0,0,1,0,1,1,1};
	int pnclrg[] = {0,0,1,0,1,1,0,1};
	int pnclrb[] = {0,1,0,0,1,0,1,1};
	int ncolor = 0;

	while(ptraj != NULL) {
		if (ptraj->m_nEFNo - ptraj->m_nSFNo + 1 >= nMinLen) {
			//printf("Traj# %d: (%d - %d)\n", ptraj->m_nTrajNo, ptraj->m_nSFNo, ptraj->m_nEFNo);
			ncolor = ptraj->m_nTrajNo % 7;
			cvClr = CV_RGB(pnclrr[ncolor]*255,pnclrg[ncolor]*255,pnclrb[ncolor]*255);
			ptrajnode = ptraj->m_pTNhead;
			nOffsetX = (ptrajnode->m_nFrameNo-nStartFrameNo)/nNumPerCol;
			if (nOffsetX % 2 == 0)
				nOffsetY = (ptrajnode->m_nFrameNo-nStartFrameNo) % nNumPerCol;
			else
				nOffsetY = nNumPerCol - 1 - ((ptrajnode->m_nFrameNo-nStartFrameNo) % nNumPerCol);
			nOffsetX *= nImgW; nOffsetY *= nImgH;
			pt1 = cvPoint( cvRound( ptrajnode->m_fx + nOffsetX ), cvRound( ptrajnode->m_fy + nOffsetY ) );
			ptrajnode = ptrajnode->m_pNext;
			while (ptrajnode != NULL) {
				nOffsetX = (ptrajnode->m_nFrameNo-nStartFrameNo)/nNumPerCol;
				if (nOffsetX % 2 == 0)
					nOffsetY = (ptrajnode->m_nFrameNo-nStartFrameNo) % nNumPerCol;
				else
					nOffsetY = nNumPerCol - 1 - ((ptrajnode->m_nFrameNo-nStartFrameNo) % nNumPerCol);
				nOffsetX *= nImgW; nOffsetY *= nImgH;
				pt2 = cvPoint( cvRound( ptrajnode->m_fx + nOffsetX ), cvRound( ptrajnode->m_fy + nOffsetY ) );

				if (ptrajnode->m_nFrameNo-nStartFrameNo >= 0) {
					cvCircle(pimgBase, pt1, 3, cvClr, -1, 8, 0);
					cvCircle(pimgBase, pt2, 3, cvClr, -1, 8, 0);
					cvLine( pimgBase, pt1, pt2, cvClr, 1, 8, 0 );
				}

				//printf("\tNode#(%d, %d) @ (%.3lf, %.3lf)\n", ptrajnode->m_nFrameNo, ptrajnode->m_nFeatureNo, ptrajnode->m_fx, ptrajnode->m_fy); 
				ptrajnode = ptrajnode->m_pNext;
				pt1 = pt2;
			}
			nCnt++;
			//ncolor = (ncolor + 1)%7;
		}
		ptraj = ptraj->m_pNext;
	}
	printf("Total: %d qualified trajectories.\n", nCnt);
}

void trajgrp::DrawFeaturesIndex(char* inFmt,double dblScale)
{
	//load the image by 'format+number' way


	char pcFNameSrc[128],pcFNameDst[128],FeatIndex[10];
	int nSrcFrameNo;
	int i;

	CvFont* myFont=new CvFont;
	cvInitFont(myFont, CV_FONT_HERSHEY_SIMPLEX , dblScale,dblScale);


	IplImage* img1,*imgout;

	trajgrp* tTrajGrp;
	traj* ptraj;
	int fx,fy;
	
	for(i=SFNo;i<=EFNo;i++)
	{
		//Load the image
		nSrcFrameNo = i;
		sprintf(pcFNameSrc, pcFNameFmt, nSrcFrameNo);

		img1=cvLoadImage(pcFNameSrc,1);
		if( ! img1 )
		{
			printf( "unable to load image from %s", pcFNameSrc );
			return;
		}
		imgout=cvCreateImage( cvSize(img1->width, img1->height), IPL_DEPTH_8U, 3 );
		cvCopy(img1,imgout);
		sprintf(pcFNameDst, inFmt, nSrcFrameNo);
		
		//Get the feature on ith frame
		tTrajGrp=GetScreenFeatureByFrame( this,i,i);
		ptraj=tTrajGrp->m_ptrajHead;
		//write all the features
		while(ptraj!=NULL)
		{
			fx=ptraj->m_pTNhead->m_fx;
			fy=ptraj->m_pTNhead->m_fy;
			sprintf(FeatIndex,"f_%d",ptraj->m_nTrajNo);
			//Writing feature index;
			cvPutText( imgout,FeatIndex,cvPoint(fx,fy),myFont, cvScalar(255,0,0));	
			cvCircle( imgout,cvPoint(fx,fy),2, cvScalar(0,255,255),-1);
			ptraj=ptraj->m_pNext;
		}
		tTrajGrp->TrajRelease();//release the group
		cvSaveImage(pcFNameDst, imgout);//same image
		printf("%s saved\n",pcFNameDst);
		cvReleaseImage(&img1);
		cvReleaseImage(&imgout);
	}

}

void trajgrp::GetFeatureColor()
{
	char pcFNameSrc[128];
	int nSrcFrameNo;
	int i,j;

	IplImage* img1;
	CvScalar s;

	trajgrp* tTrajGrp;
	traj* ptraj;
	traj* pCur;
	int fx,fy;
	
	for(i=SFNo;i<=EFNo;i++)
	{
		//Load the image
		nSrcFrameNo = i;
		sprintf(pcFNameSrc, pcFNameFmt, nSrcFrameNo);

		img1=cvLoadImage(pcFNameSrc,1);
		if( ! img1 )
		{
			printf( "unable to load image from %s", pcFNameSrc );
			return;
		}
		
		
		//Get the feature on ith frame
		tTrajGrp=GetScreenFeatureByFrame( this,i,i);
		ptraj=tTrajGrp->m_ptrajHead;
		pCur=this->m_ptrajHead;
		//write all the features
		while(ptraj!=NULL)
		{
			fx=(int)(ptraj->m_pTNhead->m_fx);
			fy=(int)(ptraj->m_pTNhead->m_fy);
			s=cvGet2D(img1,fy,fx);//coordinate switch?
			//Writing feature index;

			while( pCur->m_nTrajNo< ptraj->m_nTrajNo)
				pCur=pCur->m_pNext;

			for(j=0;j<img1->nChannels;j++)
				pCur->RGB[j] += s.val[j]/(pCur->m_nEFNo-pCur->m_nSFNo+1);
			
			ptraj=ptraj->m_pNext;
		}
		tTrajGrp->TrajRelease();//release the group		
		cvReleaseImage(&img1);		
	}
}
int trajgrp::outFeatureColor(char* pcFName)
{
	FILE* fpOut;
	if ((fpOut = fopen(pcFName, "wt")) == NULL) {
		printf("Unable to write trajlist file %s.\n", pcFName);
		return 0;
	}

	traj* ptraj = m_ptrajHead;
	trajnode* ptrajnode;
	ptraj = m_ptrajHead;

	fprintf(fpOut,"START_FEATURE_COLOR\n");
	fprintf(fpOut,"FileName=%s SF=%d EF=%d\n",pcFNameFmt,SFNo,EFNo);
	while(ptraj != NULL) 
	{
		fprintf(fpOut, "TrajNo=%d RGB=%d;%d;%d\n", 
			ptraj->m_nTrajNo, (int)(ptraj->RGB[0]),(int)(ptraj->RGB[1]),(int)(ptraj->RGB[2]));				
		ptraj = ptraj->m_pNext;
	}
	fclose(fpOut);
	return 1;

}
int trajgrp::GetFeatureColor(char* pcFName)
{
	
	FILE* fpIn;
	if((fpIn=fopen(pcFName,"r"))==NULL)
	{
		printf("Unable to read the trajlist file %s.\n",pcFName);
		return 0;
	}
	char strCmd[20];
	fscanf(fpIn,"%s\n",strCmd);
	if(strcmp(strCmd,"START_FEATURE_COLOR")!=0)
	{
		printf("not a feature color file");
		return 0;
	}
	int tSFNo,tEFNo;
	char *tpcFNameFmt=new char[50];
	fscanf(fpIn,"FileName=%s SF=%d EF=%d\n",tpcFNameFmt,&tSFNo,&tEFNo);
	if(strcmp(tpcFNameFmt,this->pcFNameFmt)!=0 || tSFNo!=this->SFNo || tEFNo!=this->EFNo)
	{
		printf("not the corresponding feature file");
		return 0;
	}
	
	traj* ptraj;
	//trajnode* ptrajnode;

	int tTrajNo;
	int i;
	int tRGB[3];

	ptraj=this->m_ptrajHead;
	while(fscanf(fpIn,"TrajNo=%d RGB=%d;%d;%d\n",&tTrajNo,&tRGB[0],&tRGB[1],&tRGB[2])!=EOF)
	{	
		while(tTrajNo > ptraj->m_nTrajNo)
		{	
			if(ptraj->m_pNext==NULL)
				break;
			ptraj=ptraj->m_pNext;
		}
		ptraj->RGB[0]=(double)tRGB[0];
		ptraj->RGB[1]=(double)tRGB[1];
		ptraj->RGB[2]=(double)tRGB[2];
	}	

	fclose(fpIn);
	return 1;

}
void trajgrp::DrawFeaturesOfOneImg(char *fileName,int ImgNo)
{

	if (ImgNo<SFNo) return;
	if (ImgNo>EFNo) return;

	CvFont* myFont=new CvFont;
	char pcFNameSrc[128],FeatIndex[128];
	IplImage *OutputImg;
	int fx,fy;
	
	cvInitFont(myFont, CV_FONT_HERSHEY_SIMPLEX , 0.3,0.3);

	sprintf(pcFNameSrc, pcFNameFmt, ImgNo);
	OutputImg = cvLoadImage(pcFNameSrc,1);

	trajgrp* tTrajGrp=GetScreenFeatureByFrame(this,ImgNo,ImgNo);
	traj* ptraj=tTrajGrp->m_ptrajHead;
		while(ptraj!=NULL)
		{
			fx=ptraj->m_pTNhead->m_fx;
			fy=ptraj->m_pTNhead->m_fy;
			sprintf(FeatIndex,"%d",ptraj->m_nTrajNo);
			//Writing feature index;
			cvPutText( OutputImg,FeatIndex,cvPoint(fx,fy),myFont, cvScalar(255,0,0));	
			cvCircle( OutputImg,cvPoint(fx,fy),2, cvScalar(0,255,255),-1);
			ptraj=ptraj->m_pNext;
		}	
	tTrajGrp->TrajRelease();
	cvSaveImage(fileName,OutputImg);
	cvReleaseImage(&OutputImg);

	delete myFont;
}

void trajgrp::CompareIndexWithOther(struct trajgrp *comtrajgrp,int **table1,int **table2,int *num)
{
	int i,j,match_count=0,icount,jcount;
	int *indMap1,*indMap2;
	traj* ptraj;
	
	icount = 0;  ptraj = this->m_ptrajHead; 
	while(ptraj!=NULL)  {	GO_m_pNext(ptraj); icount++; 	}
	jcount = 0;  ptraj = comtrajgrp->m_ptrajHead; 
	while(ptraj!=NULL)  {	GO_m_pNext(ptraj); jcount++; 	}

	indMap1 = new int[icount];
	indMap2 = new int[jcount];
	icount = 0;  ptraj = this->m_ptrajHead; 
	while(ptraj!=NULL)  
	{	indMap1[icount] = ptraj->m_nTrajNo;
		GO_m_pNext(ptraj); icount++;	}

	jcount = 0;  ptraj = comtrajgrp->m_ptrajHead; 
	while(ptraj!=NULL)  
	{	indMap2[jcount] = ptraj->m_nTrajNo;
		GO_m_pNext(ptraj); jcount++;	}

	num[0] = icount < jcount ? icount : jcount;
	*table1 = new int[num[0]];
	*table2 = new int[num[0]];

	j=0;
	for (i=0;i<icount;)
	{
		while((i<icount)&&(indMap1[i]<indMap2[j])) i++;
		while((j<jcount)&&(indMap2[j]<indMap1[i])) j++;
		if (j>=jcount) break;
		if (i>=icount) break;
		if (indMap1[i]==indMap2[j])
		{
			(*table1)[match_count] = i;
			(*table2)[match_count] = j;
			match_count++;i++;
		}	
	}
	delete [] indMap1;
	delete [] indMap2;
	num[0] = match_count;
}