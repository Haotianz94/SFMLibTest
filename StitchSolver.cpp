#include "StitchSolver.h"
#include "CV_Util2.h"
#include <direct.h>
#include "Logger.h"
#include "Configer.h"
#include <fstream>

using namespace std;

StitchSolver::StitchSolver()
{
	string baseFolder, folder, No, Cmvs;
	Configer::getConfiger()->getString("input", "baseFolder", baseFolder);
	Configer::getConfiger()->getString("input", "folder", folder);
	Configer::getConfiger()->getString("input", "No", No);
	Configer::getConfiger()->getString("input", "Cmvs", Cmvs);
	
	REPORT(baseFolder);
	REPORT(folder);
	REPORT(No);
	REPORT(Cmvs);

	oriFolder = baseFolder + folder + string("\\") + No + string("\\origin");
	maskFolder = baseFolder + folder + string("\\") + No + string("\\mask");
	fgFolder = baseFolder + folder + string("\\") + No + string("\\fg");
	bgFolder = baseFolder + folder + string("\\") + No + string("\\bg");
	siftFolder = baseFolder + folder + string("\\") + No + string("\\sift");
	outFolder = baseFolder + folder + string("\\") + No + string("\\out");
	cameraList = baseFolder + folder + string("\\") + No + string("\\") + Cmvs + string("\\00\\cameras_v2.txt");
	bundlerRes = baseFolder + folder + string("\\") + No + string("\\") + Cmvs + string("\\00\\bundle.rd.out");

	Configer::getConfiger()->getInt("warp", "GridX", GridX);
	Configer::getConfiger()->getInt("warp", "GridY", GridY);
	REPORT(GridX);
	REPORT(GridY);


	//create folder
	_mkdir(fgFolder.c_str());
	_mkdir(bgFolder.c_str());
	_mkdir(siftFolder.c_str());
	_mkdir(outFolder.c_str());
	_mkdir(join_path("warpedFG1").c_str());
	_mkdir(join_path("warpedFG2").c_str());
	_mkdir(join_path("warpedBG").c_str());
	_mkdir(join_path("match").c_str());
	_mkdir(join_path("mesh").c_str());

}

void StitchSolver::prepareForBundler()
{
	// Handle mask before recovering
	SIFTHandle::updateSIFTfolder(oriFolder, maskFolder, fgFolder, bgFolder, siftFolder);

}

void StitchSolver::loadReconstruction()
{
	recover.sfmLoader.init(cameraList, bundlerRes);
	recover.getFilePaths(maskFolder, fgFolder);
	recover.mapCam2Frame();
	recover.createNewCamPath();
	FrameH = recover.FrameH;
	FrameW = recover.FrameW;
	
	int testFrameNum = 30;
	Configer::getConfiger()->getInt("input", "testFrameNum", testFrameNum);
	for(int i = 0; i < testFrameNum; i++)
		recover.matchedFramesID.push_back(pair<int, int>(i, i));
	recover.cam1Num = recover.cam2Num = testFrameNum;
	//recover.matchedFramesID.push_back(pair<int, int>(0, 0));
	//set lackFramePair
	for(int i = 34; i < 78; i++)
		recover.lackFramesPair.push_back(i);

	bool warpFG;
	Configer::getConfiger()->getBool("input", "warpFG", warpFG);
	if(warpFG)
		recover.getForeGround3DAllFrames();
	
	recover.getBackGround3DAllFrames();

	//prepare images for paper
	/*
	Mat fg = imread(recover.cam2ImgNames[0]);
	Mat bg = fg.clone();
	for(auto& pos : recover.allForeGroundScenePoints[0])
		circle(fg, pos.pos2D_2, 6, Scalar(0, 0, 255), -1);
	for(auto& pos: recover.allBackGroundPointsCam2[0])
		circle(bg, pos.pos2D, 6, Scalar(0, 255, 0), -1);
	string fg_path = outFolder + string("\\fg.jpg");
	string bg_path = outFolder + string("\\bg.jpg");
	imwrite(fg_path.c_str(), fg);
	imwrite(bg_path.c_str(), bg);

	Mat bg;
	for(int i = 0; i < 5; i++)
	{
		bg = imread(recover.cam2ImgNames[i]);
		for(auto& pos: recover.allBackGroundPointsCam2[i])
			circle(bg, pos.pos2D, 6, Scalar(0, 255, 0), -1);
		char bg_path[100];
		sprintf_s(bg_path, "%s\\bg%d.jpg", outFolder.c_str(), i);
		imwrite(bg_path, bg);
	}
	*/
}

void StitchSolver::warpFGOnMesh(bool isSequence1)
{

	LOG << "Warping foreground based on mesh...\n\n";

	vector<Mat_<Vec2f>> deformedMesh;
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		warpFGOnMesh(i, isSequence1, deformedMesh);
		LOG << "Calculating deformed mesh for " << i << "th frame finished\n\n";
	}

	//run filter in deformed meshes
	fillMissedMesh(isSequence1, deformedMesh);
	filterDeformedMesh(deformedMesh);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{	
		int frameId1 = recover.matchedFramesID[i].first;
		int frameId2 = recover.matchedFramesID[i].second;
		int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
		int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];
		Mat origin;
		if(isSequence1)
			origin = imread(recover.cam1ImgNames[frameId1]);
		else
			origin = imread(recover.cam2ImgNames[frameId2]);
		Mat warped(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
		
		//draw deformed mesh
		/*
		vector<Point2f> oriPoints;//Here is NULL
		CVUtil::visualizeMeshAndFeatures(origin, warped, oriPoints, deformedMesh[i]);
		string baseFolder, folder, No, Cmvs;
		Configer::getConfiger()->getString("input", "baseFolder", baseFolder);
		Configer::getConfiger()->getString("input", "folder", folder);
		Configer::getConfiger()->getString("input", "No", No);
		string meshFolder = baseFolder + folder + string("\\") + No + string("\\out\\mesh\\");
		char num[10];
		sprintf_s(num, "%d", frameId1);
		imwrite((meshFolder + string(num) + string(".jpg")), warped);
		*/

		//perform mesh based warping
		warper.warpBilateralInterpolate(origin, deformedMesh[i], warped);

		char path[100];
		if(isSequence1)
			sprintf_s(path, "%s\\warpedFG1\\img%d.jpg", outFolder.c_str(), i);
		else
			sprintf_s(path, "%s\\warpedFG2\\img%d.jpg", outFolder.c_str(), i);
		imwrite(path, warped);
		
		LOG << "Warping " << i << "th frame finished\n\n";
	}
}

void StitchSolver::warpFGOnMesh(int frameMatchId, bool isSequence1, vector<Mat_<Vec2f>>& deformedMesh)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];

	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;

	//fill in foreground points

		//if there is no foreground in one frame, continue
		if( std::find(recover.lackFramesPair.begin(), recover.lackFramesPair.end(), frameMatchId) != recover.lackFramesPair.end() )
		{
			Mat_<Vec2f> noneMesh;
			deformedMesh.push_back(noneMesh);
			return;
		}
		vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
		
		for(int k = 0; k < 1; k++)
		{
			for(unsigned i = 0; i < ScnPoints.size(); i++)
			{
				if(isSequence1)
				{
					oriPoints.push_back(ScnPoints[i].pos2D_1);
					tgtPoints.push_back(ScnPoints[i].pos2D_2);
				}
				else
				{
					oriPoints.push_back(ScnPoints[i].pos2D_2);
					tgtPoints.push_back(ScnPoints[i].pos2D_1);
				}
				scnPoints.push_back(ScnPoints[i].scenePos);
			}
		}
	LOG << "Feature points after filling FG: " << (int)oriPoints.size() << '\n';

	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	//generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[idInSFM1].getMedian(recover.sfmLoader.allCameras[idInSFM2]));
	generator.getNewMesh();

	deformedMesh.push_back(generator.deformedMesh);
}

void StitchSolver::warpBGOnMesh()
{

	LOG << "Warping background based on mesh...\n\n";

	vector<Mat_<Vec2f>> deformedMesh1, deformedMesh2;
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		warpBGOnMesh(i, true, deformedMesh1);
		warpBGOnMesh(i, false, deformedMesh2);
		LOG << "Calculating deformed mesh for " << i << "th frame finished\n\n";
	}

	//run filter in deformed meshes
	filterDeformedMesh(deformedMesh1);
	filterDeformedMesh(deformedMesh2);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{	
		int frameId1 = recover.matchedFramesID[i].first;
		int frameId2 = recover.matchedFramesID[i].second;
		int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
		int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];
		
		Mat origin1, origin2;
		origin1 = imread(recover.cam1ImgNames[frameId1]);
		origin2 = imread(recover.cam2ImgNames[frameId2]);
		Mat warped1(FrameH, FrameW, CV_8UC3, Scalar(0, 0, 0));
		Mat warped2(FrameH, FrameW, CV_8UC3, Scalar(0, 0, 0));
		
		//draw deformed mesh
		/*
		vector<Point2f> oriPoints;//Here is NULL
		CVUtil::visualizeMeshAndFeatures(origin, warped, oriPoints, deformedMesh[i]);
		string baseFolder, folder, No, Cmvs;
		Configer::getConfiger()->getString("input", "baseFolder", baseFolder);
		Configer::getConfiger()->getString("input", "folder", folder);
		Configer::getConfiger()->getString("input", "No", No);
		string meshFolder = baseFolder + folder + string("\\") + No + string("\\out\\mesh\\");
		char num[10];
		sprintf_s(num, "%d", frameId1);
		imwrite((meshFolder + string(num) + string(".jpg")), warped);
		*/

		//perform mesh based warping
		warper.warpBilateralInterpolate(origin1, deformedMesh1[i], warped1);
		warper.warpBilateralInterpolate(origin2, deformedMesh2[i], warped2);

		//Todo: blend two backgrounds

		Mat warped(FrameH*2, FrameW*2, CV_8UC3, Scalar(0, 0, 0));
		char path[100];
		sprintf_s(path, "%s\\warpedBG\\left_img%d.jpg", outFolder.c_str(), i);
		imwrite(path, warped1);
		sprintf_s(path, "%s\\warpedBG\\right_img%d.jpg", outFolder.c_str(), i);
		imwrite(path, warped2);
		
		LOG << "Warping " << i << "th frame finished\n\n";
	}
}

void StitchSolver::warpBGOnMesh(int frameMatchId, bool isSequence1, vector<Mat_<Vec2f>>& deformedMesh)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];

	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;

	//fill in foreground points
	/*
		//if there is no foreground in one frame, continue
		if( std::find(recover.lackFramesPair.begin(), recover.lackFramesPair.end(), frameMatchId) != recover.lackFramesPair.end() )
		{
			Mat_<Vec2f> noneMesh;
			deformedMesh.push_back(noneMesh);
			return;
		}
		vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
		
		for(int k = 0; k < 1; k++)
		{
			for(unsigned i = 0; i < ScnPoints.size(); i++)
			{
				if(isSequence1)
				{
					oriPoints.push_back(ScnPoints[i].pos2D_1);
					tgtPoints.push_back(ScnPoints[i].pos2D_2);
				}
				else
				{
					oriPoints.push_back(ScnPoints[i].pos2D_2);
					tgtPoints.push_back(ScnPoints[i].pos2D_1);
				}
				scnPoints.push_back(ScnPoints[i].scenePos);
			}
		}
	LOG << "Feature points after filling FG: " << (int)oriPoints.size() << '\n';
	*/

	
	//fill in background points
		vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
		vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
		for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
		{
			if(isSequence1)
			{
				oriPoints.push_back(BGScnPointsCam1[i].pos2D);
				scnPoints.push_back(BGScnPointsCam1[i].scenePos);
			}
			else
				tgtPoints.push_back(BGScnPointsCam1[i].pos2D);
		}
		for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
		{
			if(!isSequence1)
			{
				oriPoints.push_back(BGScnPointsCam2[i].pos2D);
				scnPoints.push_back(BGScnPointsCam2[i].scenePos);
			}
			else
				tgtPoints.push_back(BGScnPointsCam2[i].pos2D);
		}
	LOG << "Feature points after filling BG: " << (int)oriPoints.size() << '\n';

	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	//generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[idInSFM1].getMedian(recover.sfmLoader.allCameras[idInSFM2]));
	generator.getNewMesh();

	deformedMesh.push_back(generator.deformedMesh);
}

void StitchSolver::fillMissedMesh(bool isSequence1, vector<Mat_<Vec2f>>& deformedMesh)
{
	LOG << "Fill missing deformed meshes...\n\n";

	//fill missing mesh
	for(int j = 0; j < recover.lackFramesPair.size(); j++)
	{
		int id = recover.lackFramesPair[j];
		int missingId = isSequence1? recover.matchedFramesID[id].first : recover.matchedFramesID[id].second;
		int last = missingId-1, next = missingId+1;
		int i = j-1;
		while( i >= 0 )
		{
			id = recover.lackFramesPair[i];
			int l = isSequence1? recover.matchedFramesID[id].first : recover.matchedFramesID[id].second;
			if(l != last)
				break;
			else
			{
				last --;
				i --;
			}
		}
		i = j+1;
		while( i < recover.lackFramesPair.size() )
		{
			id = recover.lackFramesPair[i];
			int n = isSequence1? recover.matchedFramesID[id].first : recover.matchedFramesID[id].second;
			if(n != next)
				break;
			else
			{
				next ++;
				i ++;
			}
		}

		Mat_<Vec2f> mesh(deformedMesh[0].rows, deformedMesh[0].cols, Vec2f(0, 0));
		REPORT(missingId);
		REPORT(last);
		REPORT(next);
		float span = next - last;
		mesh = deformedMesh[last] * (next-missingId)/span + deformedMesh[next] * (missingId-last)/span;
		deformedMesh[missingId] = mesh;
	}
}

void StitchSolver::filterDeformedMesh(vector<Mat_<Vec2f>>& deformedMesh)
{
	//Median filter
	int iteration = 3;
	int step = 1;
	Configer::getConfiger()->getInt("deformedMesh", "iteration", iteration);
	Configer::getConfiger()->getInt("deformedMesh", "step", step);

	for(int k = 0; k < iteration; k++)
	{
		for(int i = 0; i < deformedMesh.size(); i++)
		{
			Mat_<Vec2f> mesh(deformedMesh[0].rows, deformedMesh[0].cols, Vec2f(0, 0));
			for(int j = -step; j <= step; j++)
			{
				if(i+j < 0)
					mesh += deformedMesh.front();
				else if(i+j > (int)deformedMesh.size()-1)
					mesh += deformedMesh.back();
				else
					mesh += deformedMesh[i+j];
			}
			deformedMesh[i] = mesh / (2*step+1);
		}
	}
}

void StitchSolver::warpFgHomo()
{
	fgWarper = ForegroundWarper(FrameW, FrameH);

	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		int id1 = recover.matchedFramesID[i].first;
		int id2 = recover.matchedFramesID[i].second;

		Mat origin1 = imread(recover.cam1ImgNames[id1]);
		Mat origin2 = imread(recover.cam2ImgNames[id2]);
		Mat mask1 = imread(recover.cam1MaskNames[id1], 0);
		Mat mask2 = imread(recover.cam2MaskNames[id2], 0);

		int idInSFM1 = recover.frame2Cam[make_pair(1, id1)];
		int idInSFM2 = recover.frame2Cam[make_pair(2, id2)];
		
		CameraModel newCM = recover.sfmLoader.allCameras[idInSFM1].getMedian(recover
			.sfmLoader.allCameras[idInSFM2]);

		fgWarper.addWarpedPair(newCM, recover
			.allForeGroundScenePoints[i], origin1, origin2, mask1, mask2);

		//write to img
		string newFg1 = outFolder + string("\\newFg1.jpg");
		string newFg2 = outFolder + string("\\newFg2.jpg");
		imwrite(newFg1, fgWarper.warpedFgCam1.back());
		imwrite(newFg2, fgWarper.warpedFgCam2.back());
	}
}

string StitchSolver::join_path(const char* s)
{
	return outFolder + string("\\") + string(s);
}

void StitchSolver::preprocessMask()
{
	recover.getFilePaths(maskFolder, oriFolder);
	
	//cut region
	/*
	for(auto& imgname : recover.cam1MaskNames)
	{
		Mat img = imread(imgname);
		Mat imgROI(img, Rect(0, 520, 1080, 1400));
		imwrite(imgname, imgROI);
	}
	for(auto& imgname : recover.cam2MaskNames)
	{
		Mat img = imread(imgname);
		Mat imgROI(img, Rect(0, 520, 1080, 1400));
		imwrite(imgname, imgROI);
	}
	*/

	//fill holes and dilate to some extent
	int thresh = 100;
	int dilation_type = MORPH_ELLIPSE;
	int dilation_size = 1;
	int filter = 128;

	for(auto &m : recover.cam1MaskNames)
	{
		Mat mask = imread(m, 0);
		medianFilter(mask, filter);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		Mat filling_dst = Mat(mask.rows, mask.cols, CV_8U, Scalar(0)); 
		Mat canny_dst, dilation_dst; 

		Canny( mask, canny_dst, thresh, thresh*2, 3 );
		findContours(canny_dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		fillPoly(filling_dst, contours, Scalar(255));

		Mat element = getStructuringElement(dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ),
		Point( dilation_size, dilation_size ) );
		dilate(filling_dst, dilation_dst, element);

		imwrite(m, dilation_dst);
	}

	for(auto &m : recover.cam2MaskNames)
	{
		Mat mask = imread(m, 0);
		medianFilter(mask, filter);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		Mat filling_dst = Mat(mask.rows, mask.cols, CV_8U, Scalar(0)); 
		Mat canny_dst, dilation_dst; 

		Canny( mask, canny_dst, thresh, thresh*2, 3 );
		findContours(canny_dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		fillPoly(filling_dst, contours, Scalar(255));

		Mat element = getStructuringElement(dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ),
		Point( dilation_size, dilation_size ) );
		dilate(filling_dst, dilation_dst, element);

		imwrite(m, dilation_dst);
	}

}

void StitchSolver::preprocessOrigin()
{
	recover.getFilePaths(maskFolder, oriFolder);

	//cut region
	/*
	for(auto& imgname : recover.cam1ImgNames)
	{
		Mat img = imread(imgname);
		Mat imgROI(img, Rect(0, 520, 1080, 1400));
		imwrite(imgname, imgROI);
	}
	for(auto& imgname : recover.cam2ImgNames)
	{
		Mat img = imread(imgname);
		Mat imgROI(img, Rect(0, 520, 1080, 1400));
		imwrite(imgname, imgROI);
	}
	*/

	//histogram matching
	Mat target = imread(recover.cam1ImgNames[0]);
	Mat target_mask = imread(recover.cam1MaskNames[0], 0);
	if(target.empty() || target_mask.empty())
		return ;

	for(unsigned id = 0; id < recover.cam2ImgNames.size(); id++)
	{
		Mat img = imread(recover.cam2ImgNames[id]);
		Mat mask = imread(recover.cam2MaskNames[id], 0);
		int nrows = img.rows;
		int ncols = img.cols;

	int CI[256][3];
	int CJ[256][3];
	double PI[256][3];
	double PJ[256][3];
	uchar LUT[256][3];
	int numI = 0, numJ = 0;

	for(int i = 0; i < 256; i++)
		for(int k = 0; k < 3; k++)
		{
			CI[i][k] = 0;
			CJ[i][k] = 0;
		}

	for(int y = 0; y < nrows; y++)
		for(int x = 0; x < ncols; x++)
			for(int k = 0; k < 3; k++)
			{
				//if(mask.at<uchar>(y, x) > 127)
				{
					uchar val = img.at<Vec3b>(y, x)[k];
					CI[val][k] += 1;
					numI ++;
				}
			}

	for(int y = 0; y < target.rows; y++)
		for(int x = 0; x < target.cols; x++)
			for(int k = 0; k < 3; k++)
			{
				//if(target_mask.at<uchar>(y, x) > 127)
				{
					uchar val =  target.at<Vec3b>(y, x)[k];
					CJ[val][k] += 1;
					numJ ++;
				}
			}

	for(int i = 0; i < 256; i++)
		for(int k = 0; k < 3; k++)
		{
			if(i > 0)
			{
				CI[i][k] += CI[i-1][k];
				CJ[i][k] += CJ[i-1][k];
			}
		}
	
	for(int i = 0; i < 256; i++)
		for(int k = 0; k < 3; k++)
		{
			PI[i][k] = 1.0 * CI[i][k] / numI;
			PJ[i][k] = 1.0 * CJ[i][k] / numJ;
		}

	for(int k = 0; k < 3; k++)
	{
		uchar j = 0;
		for(int i = 0; i < 256; i++)
		{
			while(PJ[j][k] < PI[i][k] && j < 255)
				j ++;
			LUT[i][k] = j;
		}
	}

	Mat res = Mat(img.size(), CV_8UC3);
	for(int y = 0; y < nrows; y++)
		for(int x = 0; x < ncols; x++)
			for(int k = 0; k < 3; k++)
				res.at<Vec3b>(y, x)[k] = LUT[img.at<Vec3b>(y, x)[k]][k];

	REPORT(recover.cam2ImgNames[id]);
	imwrite(recover.cam2ImgNames[id], res);

	}
}

void StitchSolver::medianFilter(Mat& image, int filter)
{
	for(int x=0; x<image.cols; x++)
		for(int y=0; y<image.rows; y++)
		{
			uchar ptr = image.at<uchar>(y,x);
			if(ptr > filter)//10
				image.at<uchar>(y,x) = 255;
			else
				image.at<uchar>(y,x) = 0;
		}

		int h = image.rows;
		int w = image.cols;
		for(int x=0; x<image.cols; x++)
			for(int y=0; y<image.rows; y++)
			{
				int num = 0;
				int sum = 0;
				for(int i=-1; i<2; i++)
					for(int j=-1; j<2; j++)
						if(x+i>=0 && x+i<w && y+j>=0 && y+j<h)
						{
							uchar ptr = image.at<uchar>(y+j, x+i);
							sum ++;
							if(ptr == 0)
								num ++;
						}
				if(num > 4 || sum < 9)
				{
					image.at<uchar>(y, x) = 0; 
				}
				else
					image.at<uchar>(y, x) = 255;
			}
}

void StitchSolver::warpMLS()
{
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		Mat warped = warpMLS(i, false);
		char path[100];
		sprintf_s(path, "%s\\warped1\\img%d.jpg", outFolder.c_str(), i);
		imwrite(path, warped);

		cout << "WarpOnMesh Frame: " << i << endl;
	}
}

Mat StitchSolver::warpMLS(int frameMatchId, bool isSequence1)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];

	Mat origin, mask;
	if(isSequence1)
	{
		origin = imread(recover.cam1ImgNames[frameId1]);
		mask = imread(recover.cam1MaskNames[frameId1], 0);
	}
	else
	{
		origin = imread(recover.cam2ImgNames[frameId2]);
		mask = imread(recover.cam2MaskNames[frameId2], 0);
	}


	vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;
	
	//fill in foreground points
	for(int k = 0; k < 1; k++)
	{
		for(unsigned i = 0; i < ScnPoints.size(); i++)
		{
			if(isSequence1)
			{
				oriPoints.push_back(ScnPoints[i].pos2D_1);
				tgtPoints.push_back(ScnPoints[i].pos2D_2);
			}
			else
			{
				oriPoints.push_back(ScnPoints[i].pos2D_2);
				tgtPoints.push_back(ScnPoints[i].pos2D_1);
			}
			scnPoints.push_back(ScnPoints[i].scenePos);
		}
	}

	//fill in background points
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
	{
		if(isSequence1)
		{
			oriPoints.push_back(BGScnPointsCam1[i].pos2D);
			scnPoints.push_back(BGScnPointsCam1[i].scenePos);
		}
		else
			tgtPoints.push_back(BGScnPointsCam1[i].pos2D);
	}
	for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
	{
		if(!isSequence1)
		{
			oriPoints.push_back(BGScnPointsCam2[i].pos2D);
			scnPoints.push_back(BGScnPointsCam2[i].scenePos);
		}
		else
			tgtPoints.push_back(BGScnPointsCam2[i].pos2D);
	}

	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	
	//generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[idInSFM1].getMedian(recover.sfmLoader.allCameras[idInSFM2]));

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
	generator.warpMLS(origin, mask, out);

	return out;
}

void StitchSolver::testWarping(int frameMatchId)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];
	Mat origin = imread(recover.cam1ImgNames[frameId1]);

	vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;
	for(int k = 0; k < 20; k++)
	{
		for(unsigned i = 0; i < ScnPoints.size(); i++)
		{
			oriPoints.push_back(ScnPoints[i].pos2D_1);
			tgtPoints.push_back(ScnPoints[i].pos2D_2);
			scnPoints.push_back(ScnPoints[i].scenePos);
		}
	}

	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
	{
		oriPoints.push_back(BGScnPointsCam1[i].pos2D);
		scnPoints.push_back(BGScnPointsCam1[i].scenePos);
	}
	for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
		tgtPoints.push_back(BGScnPointsCam2[i].pos2D);

	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	//generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[idInSFM1].getMedian(recover.sfmLoader.allCameras[idInSFM2]));
	generator.getNewMesh();

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	CVUtil::visualizeMeshAndFeatures(origin, out, oriPoints, generator.deformedMesh);
	imshow("match", out);
	waitKey(0);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	warper.warpBilateralInterpolate(origin, generator.deformedMesh, out);
	imshow("warp", out);
	waitKey(0);
}
void StitchSolver::testWarping_middle(int frameMatchId)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;

	CameraModel cam1 = recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(1, frameId1)]];
	CameraModel cam2 = recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]];

	
	vector<scenePointOnPair>& ScnPoints = recover.allForeGroundScenePoints[frameMatchId];
	vector<Point2f> oriPoints, tgtPoints;
	vector<Point3f> scnPoints;
	for(int k = 0; k < 20; k++)
	{
		for(unsigned i = 0; i < ScnPoints.size(); i++)
		{
			oriPoints.push_back(ScnPoints[i].pos2D_1);
			tgtPoints.push_back(ScnPoints[i].pos2D_2);
			scnPoints.push_back(ScnPoints[i].scenePos);
		}
	}
	
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	for(unsigned i = 0; i < BGScnPointsCam1.size(); i++)
	{
		oriPoints.push_back(BGScnPointsCam1[i].pos2D);
		scnPoints.push_back(BGScnPointsCam1[i].scenePos);
	}
	for(unsigned i = 0; i < BGScnPointsCam2.size(); i++)
		tgtPoints.push_back(BGScnPointsCam2[i].pos2D);


	generator = ViewGenerator(oriPoints, tgtPoints, scnPoints, FrameW, FrameH, GridX, GridY);
	generator.getNewFeaturesPos(recover.sfmLoader.allCameras[recover.frame2Cam[make_pair(2, frameId2)]]);
	generator.getNewMesh();

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	CVUtil::visualizeMeshAndFeatures(imread(recover.cam1ImgNames[frameId1]), out, oriPoints, generator.deformedMesh);
	imshow("match", out);
	waitKey(0);

	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	warper.warpBilateralInterpolate(imread(recover.cam1ImgNames[frameId1]), generator.deformedMesh, out);
	imshow("warp", out);
	imwrite("warp.jpg", out);
	waitKey(0);
}
void StitchSolver::test()
{
	Mat img = imread(recover.cam2ImgNames[0]);
	for(auto& pos: recover.allSiftsCam2[0].allFeatsAbs)
		circle(img, pos.pos, 2, Scalar(255, 0, 0), -1);

	for(auto& pos : recover.allBackGroundPointsCam2[0])
		circle(img, pos.pos2D, 2, Scalar(0, 0, 255), -1);

	imshow("img2", img);
	waitKey(0);
}

void StitchSolver::extractFeatureFG()
{
	recover.getFilePaths(maskFolder, fgFolder); 
	/*
	for(unsigned i = 0; i < recover.cam1ImgNames.size(); i++)
	{
		Mat img = imread(recover.cam1ImgNames[i]);
		Mat mask = imread(recover.cam1MaskNames[i], 0);
		extractFeatureFG(img, mask, recover.cam1ImgNames[i]);
	}
	*/
	for(unsigned i = 30; i < recover.cam2ImgNames.size(); i++)
	{
		Mat img = imread(recover.cam2ImgNames[i]);
		Mat mask = imread(recover.cam2MaskNames[i], 0);
		extractFeatureFG(img, mask, recover.cam2ImgNames[i]);
	}
}

void StitchSolver::extractFeatureFG(Mat& img, Mat& mask, string imgPath)
{
	string featureFG;
	Configer::getConfiger()->getString("input", "featureFG", featureFG);

	initModule_nonfree(); 
	Ptr<FeatureDetector> detector = FeatureDetector::create( featureFG );
	Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create( featureFG );
	if( detector.empty() || descriptor_extractor.empty() )    
		cout << "fail to create detector!";    
		
	//detect feature position
	vector<KeyPoint> kp;    
	detector->detect( img, kp );   
	LOG << "All feature point num: " << (int)kp.size() << '\n';

	//remove feature in background
	unsigned k = 0;
	while(k < kp.size())
	{
		if(mask.at<uchar>(kp[k].pt.y, kp[k].pt.x) < 127)
			kp.erase(kp.begin() + k);
		else
			k ++;
	}
	LOG << "FG feature point num: " << (int)kp.size() << '\n';
		 
	//extract feature vector
	Mat descriptors;    
	descriptor_extractor->compute( img, kp, descriptors );    

	//write feature to file
	string featFile;
	if(featureFG == "SURF")
		featFile = imgPath.substr(0, imgPath.length()-4) + string(".surf");
	REPORT(featFile);

	ofstream fout(featFile);
	fout << descriptors.rows << endl;
	for(int i = 0; i < descriptors.rows; i++)
	{
		fout << kp[i].pt.x << ' '<< kp[i].pt.y << endl;
		for(int j = 0; j < 64; j++)
			fout << descriptors.at<float>(i, j) << endl;
	}
	fout.close();
}
