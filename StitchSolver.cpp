#include "StitchSolver.h"
#include "CV_Util2.h"

using namespace std;
StitchSolver::StitchSolver(string folder, string No)
{
	string baseFolder = string("I:\\2016fallproject\\");

	oriFolder = baseFolder + folder + string("\\") + No + string("\\origin");
	maskFolder = baseFolder + folder + string("\\") + No + string("\\mask");
	fgFolder = baseFolder + folder + string("\\") + No + string("\\fg");
	bgFolder = baseFolder + folder + string("\\") + No + string("\\bg");
	outFolder = baseFolder + folder + string("\\") + No + string("\\out");
	//siftMaskResFolder = string("I:\\2016fallproject\\data2\\video001_002_recon\\");
	cameraList = baseFolder + folder + string("\\") + No + string("\\all.nvm.cmvs\\00\\cameras_v2.txt");
	bundlerRes = baseFolder + folder + string("\\") + No + string("\\all.nvm.cmvs\\00\\bundle.rd.out");
	GridX = 20;
	GridY = 20;
}

void StitchSolver::prepareForBundler()
{
	// Handle mask before recovering
	SIFTHandle::updateSIFTfolder(oriFolder, maskFolder, fgFolder, bgFolder);

}

void StitchSolver::loadReconstruction()
{
	recover.sfmLoader.init(cameraList, bundlerRes);
	recover.getFilePaths(maskFolder, fgFolder);

	recover.mapCam2Frame();
	FrameH = recover.FrameH;
	FrameW = recover.FrameW;
	
	for(int i = 0; i < 1; i++)
		recover.matchedFramesID.push_back(pair<int, int>(i, i));
	//recover.matchedFramesID.push_back(pair<int, int>(0, 0));
	
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

void StitchSolver::warpOnMesh()
{
	for(int i = 0; i < recover.matchedFramesID.size(); i++)
	{
		Mat warped = warpOnMesh(i, true);
		char path[100];
		sprintf_s(path, "%s\\warped1\\img%d.jpg", outFolder.c_str(), i);
		imwrite(path, warped);

		cout << "WarpOnMesh Frame: " << i << endl;
	}
}

Mat StitchSolver::warpOnMesh(int frameMatchId, bool isSequence1)
{
	int frameId1 = recover.matchedFramesID[frameMatchId].first;
	int frameId2 = recover.matchedFramesID[frameMatchId].second;
	int idInSFM1 = recover.frame2Cam[make_pair(1, frameId1)];
	int idInSFM2 = recover.frame2Cam[make_pair(2, frameId2)];

	Mat origin;
	if(isSequence1)
		origin = imread(recover.cam1ImgNames[frameId1]);
	else
		origin = imread(recover.cam2ImgNames[frameId2]);


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
	
	REPORT(oriPoints.size());

	//fill in background points
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
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
	generator.getNewMesh();

	Mat out(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	CVUtil::visualizeMeshAndFeatures(origin, out, oriPoints, generator.deformedMesh);
	//imshow("match", out);
	//waitKey(0);
	imwrite(join_path("mesh.jpg").c_str(), out);


	warper = Warper(Mesh(FrameW, FrameH, GridX, GridY));
	warper.warpBilateralInterpolate(origin, generator.deformedMesh, out);
	//imshow("warp", out);
	//waitKey(0);
	return out;
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
	int thresh = 100;
	int dilation_type = MORPH_ELLIPSE;
	int dilation_size = 10;
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
	
	REPORT(oriPoints.size());

	//fill in background points
	/*
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
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
	*/

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
	/*
	for(int k = 0; k < 20; k++)
	{
		for(unsigned i = 0; i < ScnPoints.size(); i++)
		{
			oriPoints.push_back(ScnPoints[i].pos2D_1);
			tgtPoints.push_back(ScnPoints[i].pos2D_2);
			scnPoints.push_back(ScnPoints[i].scenePos);
		}
	}
	REPORT(oriPoints.size());
	*/
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
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
	
	REPORT(oriPoints.size());
	vector<scenePoint> BGScnPointsCam1 = recover.allBackGroundPointsCam1[frameId1];
	vector<scenePoint> BGScnPointsCam2 = recover.allBackGroundPointsCam2[frameId2];
	REPORT(BGScnPointsCam1.size());
	REPORT(BGScnPointsCam2.size());
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