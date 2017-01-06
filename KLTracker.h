//KTL include headerfile
#if !defined(___KLT_TRACKER____H__)
#define  ___KLT_TRACKER____H__


#ifndef GO_NEXT
#define GO_NEXT(set)\
        set = set->next
#endif


typedef class IMGList IMGList;
typedef class FeatureVector FeatureVector ;

class IMGList
{
	public:
		int nCols,nRows;
		unsigned char *Img;  //RGB data
		IMGList *next;
		IMGList() {  nCols=0; nRows=0; Img = 0; next = 0; }
		IMGList(int A,int B,int ColorChannel) {  nCols=A; nRows=B; Img = new unsigned char[A*B*ColorChannel]; next = 0;}
		~IMGList() {  if (Img) delete[] Img;}

};


//each FeatureVector indicates All feature in one frame and corresponding to successive frames 
class FeatureVector
{
	public:
		float **vertex;
		int *isValid,size;
		FeatureVector() {  isValid = 0; vertex = 0; size=0; };
		FeatureVector(int assignedSize)  { Create(assignedSize); };
		void Create(int assignedSize) 
		{   int i;
		    if (assignedSize>0)
			{
				size = assignedSize;
				isValid = new int[assignedSize];
				for (i=0;i<size;i++) isValid[i] = 0;
				
				vertex = new float*[size];
				for (i=0;i<size;i++) 
				{
					vertex[i] = new float[2];
					vertex[i][0] = 0;
					vertex[i][1] = 0;
				}
			}
		};
		
		~FeatureVector() 
		{   
			int i;
			if (isValid) delete[] isValid;
			
			if ((vertex)&&(size>0))
			{
				for (i=0;i<size;i++)
					delete[] vertex[i];
				delete[] vertex;
			}
		};
		
};


FeatureVector *KLTFeatureRetrievalRGB(unsigned char **RGBArray,int nCols,int nRows, int nFrames,int nFeatures);
FeatureVector *KLTFeatureRetrievalGrey(unsigned char **GreyArray,int nCols,int nRows, int nFrames,int nFeatures);
FeatureVector *KLTFeatureRetrievalList(IMGList *RGBImgList,int nCols,int nRows, int nFrames,int nFeatures);
//Others


FeatureVector *KLTFeatureRetrievalListMin(IMGList *RGBImgList,int nCols,int nRows, int nFrames,int nFeatures,int nMinFeatures);
//Main KLT Function

#endif
