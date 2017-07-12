#include "Stabilizer.h"
using namespace std;
using namespace cv;

Point2f transform(Mat &H, Point2f srcp)
{
	double src[3] = { srcp.x, srcp.y, 1 };
	double dest[3] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			dest[i] += H.at<double>(i, j) * src[j];
	return Point2f(dest[0] / dest[2], dest[1] / dest[2]);
}

Vec3b biliner_interpolate(Point2f pos, Mat &img)
{
	float X = pos.x;
	float Y = pos.y;

	if (X < 0 || X >= img.cols - 1 || Y < 0 || Y >= img.rows - 1)
		return Vec3b(255, 255, 255);

	Vec3b p, q, r;
	p = img.at<Vec3b>((int)Y, (int)X) + (X - (int)X) * (img.at<Vec3b>((int)Y, (int)X + 1) - img.at<Vec3b>((int)Y, (int)X));
	q = img.at<Vec3b>((int)Y + 1, (int)X) + (X - (int)X) * (img.at<Vec3b>((int)Y + 1, (int)X + 1) - img.at<Vec3b>((int)Y + 1, (int)X));
	r = p + (Y - (int)Y) * (q - p);

	return r;
}
Stabilizer::Stabilizer()
{
}


Stabilizer::Stabilizer(std::string folder, std::string No)
{
	string baseFolder = string("D:/AndroidProgramme/newData/");

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

Stabilizer::~Stabilizer()
{
}

void Stabilizer::prepareForBundler()
{
	// Handle mask before recovering
//	SIFTHandle::updateSIFTfolder(oriFolder, maskFolder, fgFolder, bgFolder);

}

void Stabilizer::loadReconstruction()
{
	recover.sfmLoader.init(cameraList, bundlerRes);
	recover.getFilePaths(maskFolder, fgFolder);

	recover.mapCam2Frame();
	FrameH = recover.FrameH;
	FrameW = recover.FrameW;

	for (int i = 0; i < 30; i++)
		recover.matchedFramesID.push_back(pair<int, int>(i, i));
	//recover.matchedFramesID.push_back(pair<int, int>(0, 0));

	recover.getForeGround3DAllFrames();
	recover.getBackGround3DAllFrames();
}

void Stabilizer::warpFgHomo()
{
	fgWarper = ForegroundWarper(FrameW, FrameH);

	for (int i = 0; i < recover.matchedFramesID.size(); i++)
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


void Stabilizer::warpOneFrame(CameraModel& newCM, vector<scenePointOnPair>& fgScene, vector<int>& select, Mat origin, Mat mask, bool isSequence1)
{
	//calculate key points for warping
/*
	vector<Point2f> kp1;
	vector<Point2f> kp2;
	Mat canvas = Mat(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
	for (unsigned i = 0; i < select.size(); i++)
	{
		Point2f originP = isSequence1 ? fgScene[select[i]].pos2D_1 : fgScene[select[i]].pos2D_2;
		kp1.push_back(originP);

		Point2f warpedP = CameraHelper::get2DViewPosition(newCM, fgScene[select[i]].scenePos);
		warpedP.x += FrameW / 2;
		warpedP.y = FrameH / 2 - warpedP.y;
		kp2.push_back(warpedP);

		line(canvas, originP, warpedP, Scalar(255, 0, 0), 1);
		circle(canvas, originP, 2, Scalar(0, 255, 0), -1);
		circle(canvas, warpedP, 2, Scalar(0, 0, 255), -1);
	}
	imshow("warped pair", canvas);
	waitKey(0);

	//warp using bilinear interpolation
	Mat H = findHomography(kp2, kp1);
	Mat newFg = Mat(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));

	int x2 = 0, x1 = FrameW, y2 = 0, y1 = FrameH;
	for (auto& p : kp2)
	{
		if (p.x < x1)
			x1 = p.x;
		if (p.x > x2)
			x2 = p.x;
		if (p.y < y1)
			y1 = p.y;
		if (p.y > y2)
			y2 = p.y;
	}

	x1 -= 50;
	x2 += 50;
	y1 -= 50;
	y2 += 50;

	if (x1 < 0)
		x1 = 0;
	if (x2 >= FrameW)
		x2 = FrameW - 1;
	if (y1 < 0)
		y1 = 0;
	if (y2 >= FrameH)
		y2 = FrameH - 1;

	for (int y = 0; y <= FrameH - 1; y++)
		for (int x = 0; x <= FrameW - 1; x++)
		{
		Point2f q = transform(H, Point(x, y));
		if (!((x>0) && (x<W) && (y>0) && (y<H)))
			continue;
		if (mask.at<uchar>(q.y, q.x) > 127)
		{
			newFg.at<Vec3b>(y, x) = biliner_interpolate(q, origin);
		}
		}

	Mat img;
	resize(newFg, img, cvSize(FrameW / 2, FrameH / 2));
	imshow("newFg", img);
	waitKey(0);*/
//	return newFg;

}

void Stabilizer::getSmoothPath()
{
	/*fgWarper = ForegroundWarper(FrameW, FrameH);
	vector<CameraModel> newPath;
	for (int i = 0; i < recover.cam1Num - 1; i++)
	{
		int idInSFM1 = recover.frame2Cam[make_pair(1, 1)];
		int idInSFM2 = recover.frame2Cam[make_pair(1, 2)];

		CameraModel newCM = recover.sfmLoader.allCameras[idInSFM1].getMedian(recover
			.sfmLoader.allCameras[idInSFM2]);

		vector<Point2f> kp1, kp2;
		for (int n = 0; n < recover.allBackGroundPointsCam1[i].size())
		{
		}
	}*/
}

