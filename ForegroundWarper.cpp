#include "ForegroundWarper.h"
#include "CameraHelper.h"
#include "Logger.h"

using namespace std;
using namespace cv;

#define BOUNDED(x, y, W, H) ( (x>0) && (x<W) && (y>0) && (y<H))


void ForegroundWarper::addWarpedPair(CameraModel& newCM, vector<scenePointOnPair>& fgScene, Mat origin1, Mat origin2, Mat mask1, Mat mask2)
{
	vector<Point2f> kp1, kp2;
	for(unsigned i = 0; i < fgScene.size(); i++)
	{
		kp1.push_back(fgScene[i].pos2D_1);
		kp2.push_back(fgScene[i].pos2D_2);
	}

		/*
		//RANSEC calculate H
		const int MAX_ITERATION = 10000;
		const int SAMPLE_NUM = 4;
		const int MIN_INLIER_NUM = 50;
		const double MAX_ERROR = 10;//3

		vector<int> best_consensus_set;
		double best_error = 1e10;
		Mat best_H;
		int Num = kp1.size();
		srand(time(0));
		for(int i = 0; i < MAX_ITERATION; i++)
		{
			//train model use random samples
			vector<int> consensus_set;
			vector<Point2f> sel_kp1, sel_kp2;
			for(int j = 0; j < SAMPLE_NUM; j++)
			{
				int index = rand()%Num;
				//cout<<index<<endl;
				sel_kp1.push_back(kp1[index]);
				sel_kp2.push_back(kp2[index]);
			}
			Mat H = findHomography(sel_kp1, sel_kp2);
			//search inlier
			for(unsigned j = 0; j < Num; j++)
			{
				Point2f p2_t = transform(H, kp1[j]);
				Point2f p2 = kp2[j];
				double error = (p2.x - p2_t.x) * (p2.x - p2_t.x) + (p2.y - p2_t.y) * (p2.y - p2_t.y);
				error = sqrt(error);
				if(error < MAX_ERROR)
					consensus_set.push_back(j);
			}
			//check model
			//cout<<"size "<<consensus_set.size()<<endl;
			if(consensus_set.size() < MIN_INLIER_NUM)
				continue;
			sel_kp1.clear();
			sel_kp2.clear();
			for(unsigned j = 0; j < consensus_set.size(); j++)
			{
				sel_kp1.push_back(kp1[consensus_set[j]]);
				sel_kp2.push_back(kp2[consensus_set[j]]);
			}
			Mat better_H = findHomography(sel_kp1, sel_kp2);
			double better_error = 0;
			for(unsigned j = 0; j < consensus_set.size(); j++)
			{
				Point2f p2_t = transform(better_H, kp1[consensus_set[j]]);
				Point2f p2 = kp2[consensus_set[j]];
				double error = (p2.x - p2_t.x) * (p2.x - p2_t.x) + (p2.y - p2_t.y) * (p2.y - p2_t.y);
				error = sqrt(error);
				better_error += error;
			}
			//cout<<"error "<<better_error<<endl;
			if(better_error < best_error)
			{
				best_error = better_error;
				best_H = better_H;
				best_consensus_set = consensus_set;
			}
		}
		*/

	REPORT((int)kp1.size());
	Mat H = findHomography(kp1, kp2);
	vector<int> best_consensus_set;
	const double MAX_ERROR = 500;
	for(unsigned i = 0; i < fgScene.size(); i++)
	{
		Point2f p2_t = transform(H, kp1[i]);
		Point2f p2 = kp2[i];
		double error = (p2.x - p2_t.x) * (p2.x - p2_t.x) + (p2.y - p2_t.y) * (p2.y - p2_t.y);
		REPORT(error);
		if(error < MAX_ERROR)
			best_consensus_set.push_back(i);
	}

	REPORT((int)best_consensus_set.size());
	Mat newFg1 = warpFgHomo(newCM, fgScene, best_consensus_set, origin1, mask1, true);
	Mat newFg2 = warpFgHomo(newCM, fgScene, best_consensus_set, origin2, mask2, false);
	warpedFgCam1.push_back(newFg1);
	warpedFgCam2.push_back(newFg2);
}

Mat ForegroundWarper::warpFgHomo(CameraModel& newCM, vector<scenePointOnPair>& fgScene, vector<int>& select, Mat origin, Mat mask, bool isSequence1)
{
	//calculate key points for warping
	vector<Point2f> kp1;
	vector<Point2f> kp2;
	Mat canvas = Mat(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
	for(unsigned i = 0; i < select.size(); i++)
	{
		Point2f originP = isSequence1? fgScene[select[i]].pos2D_1 : fgScene[select[i]].pos2D_2;   
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

	/*
	int x2 = 0, x1 = FrameW, y2 = 0, y1 = FrameH;
	for(auto& p : kp2)
	{
		if(p.x < x1)
			x1 = p.x;
		if(p.x > x2)
			x2 = p.x;
		if(p.y < y1)
			y1 = p.y;
		if(p.y > y2)
			y2 = p.y;
	}

	x1 -= 50;
	x2 += 50;
	y1 -= 50;
	y2 += 50;
	
	if(x1 < 0)
		x1 = 0;
	if(x2 >= FrameW)
		x2 = FrameW - 1;
	if(y1 < 0)
		y1 = 0;
	if(y2 >= FrameH)
		y2 = FrameH - 1;
	*/

	for(int y = 0; y <= FrameH-1; y++)
		for(int x = 0; x <= FrameW-1; x++)
		{
			Point2f q = transform(H, Point(x, y));
			if(!BOUNDED(q.x, q.y, FrameW, FrameH))
				continue;
			if(mask.at<uchar>(q.y, q.x) > 127)
			{
				newFg.at<Vec3b>(y, x) = biliner_interpolate(q, origin);
			}
		}

	Mat img;
	resize(newFg, img, cvSize(FrameW/2, FrameH/2));
	imshow("newFg", img);
	waitKey(0);
	return newFg;
}

Point2f ForegroundWarper::transform(Mat &H, Point2f srcp)
{
	double src[3] = {srcp.x, srcp.y, 1};
	double dest[3] = {0, 0, 0};
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			dest[i] += H.at<double>(i,j) * src[j];
	return Point2f(dest[0]/dest[2], dest[1]/dest[2]);
}

Vec3b ForegroundWarper::biliner_interpolate(Point2f pos, Mat &img)
{
	float X = pos.x;
	float Y = pos.y;
	
	if(X < 0 || X >= img.cols-1 || Y < 0 || Y >= img.rows-1)
		return Vec3b(255, 255, 255);

	Vec3b p,q,r;
	p = img.at<Vec3b>((int)Y, (int)X) + (X - (int)X) * (img.at<Vec3b>((int)Y ,(int)X + 1) - img.at<Vec3b>((int)Y, (int)X));
	q = img.at<Vec3b>((int)Y + 1,(int)X) + (X - (int)X) * (img.at<Vec3b>((int)Y + 1, (int)X + 1) - img.at<Vec3b>((int)Y + 1,(int)X));
	r = p + (Y - (int)Y) * (q - p);
	
	return r;
}