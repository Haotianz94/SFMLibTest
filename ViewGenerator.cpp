#include "ViewGenerator.h"
#include "Warp.h"

#define BOUNDED(x, y, W, H) ( (x>0) && (x<W) && (y>0) && (y<H))

void ViewGenerator::getNewFeaturesPos(CameraModel& newViewCM)
{
	for (int i = 0; i < ScenePoints3D.size(); i++)
	{
		cv::Point2f pos2d = CameraHelper::get2DViewPosition(newViewCM, ScenePoints3D[i]);
		pos2d.x += FrameW / 2;
		pos2d.y = FrameH / 2 - pos2d.y;

		tgtPoints.push_back(pos2d);
	}

	//draw feature 2D for 2 old views and 1 new view 
	/*
	Mat img(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
	for(unsigned i = 0; i < tgtPoints.size(); i++)
	{ 
		circle(img, warp(tgtPoints[i]), 1, Scalar(0, 0, 255), -1);
		circle(img, warp(oriPoints[i]), 1, Scalar(0, 255, 0), -1);
	}
	for(unsigned i = 0; i < tgtPoints_real.size(); i++)
		circle(img, warp(tgtPoints_real[i]), 1, Scalar(255, 0, 0), -1);
	imshow("trans", img);
	waitKey(0);
	*/
}

void ViewGenerator::getNewMesh()
{
	ASAPSolver asap(FrameW, FrameH, GridX, GridY);
	asap.solve(oriPoints, tgtPoints, this->deformedMesh);

	//draw new Mesh
	/*
	Mat newMesh(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
	for(int y = 0; y < deformedMesh.rows; y++)
		for(int x = 0; x < deformedMesh.cols; x++)
		{
			Vec2f pos = deformedMesh.at<Vec2f>(y, x);
			circle(newMesh, Point(pos[0], pos[1]), 2, Scalar(0, 0, 0), -1);
		}
	imshow("newMesh", newMesh);
	waitKey(0);
	*/
}

void ViewGenerator::warpMLS(Mat& origin, Mat& mask, Mat& out)
{
	vector<CvPoint2D32f> src_points;
	vector<CvPoint2D32f> dest_points;
	
	for(auto& pos : oriPoints)
	{
		src_points.push_back(Point(pos.x, pos.y));
	}
	for(auto& pos : tgtPoints)
	{
		dest_points.push_back(Point(pos.x, pos.y));
	}

	CWarp* m_warping = new CMLS(dest_points, src_points);//???


	//warp using bilinear interpolation

	for(int y = 0; y <= FrameH-1; y++)
		for(int x = 0; x <= FrameW-1; x++)
		{
			if(x == FrameW-1)
				cout << y << endl;

			CvPoint2D32f q = m_warping->Warping(Point(x, y));
			if(!BOUNDED(q.x, q.y, FrameW, FrameH))
				continue;
			if(mask.at<uchar>(q.y, q.x) > 127)
			{
				out.at<Vec3b>(y, x) = biliner_interpolate(q, origin);
			}
		}
}

Vec3b ViewGenerator::biliner_interpolate(Point2f pos, Mat &img)
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


ViewGenerator::ViewGenerator()
{
}


ViewGenerator::ViewGenerator(vector<Point2f>& oriPoints2D, vector<Point2f>& tgtPoints2D, vector<Point3f>& oriPoints3D, int FrameWidth /*= 1920*/, int FrameHeight /*= 1080*/, int GridXNum /*= 20*/, int GridYNum /*= 20*/)
{
	GridX = GridXNum;
	GridY = GridYNum;
	FrameW = FrameWidth;
	FrameH = FrameHeight;
	for (int i = 0; i < oriPoints2D.size(); i++)
	{
		oriPoints.push_back(oriPoints2D[i]);
		if (i < tgtPoints2D.size())
			tgtPoints_real.push_back(tgtPoints2D[i]);
		ScenePoints3D.push_back(oriPoints3D[i]);
	}
}

ViewGenerator::~ViewGenerator()
{
}
