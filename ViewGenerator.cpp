#include "ViewGenerator.h"


void ViewGenerator::getNewFeaturesPos(CameraModel& newViewCM)
{
	for (int i = 0; i < ScenePoints3D.size(); i++)
	{
		cv::Point2f pos2d = CameraHelper::get2DViewPosition(newViewCM, ScenePoints3D[i]);
		pos2d.x += FrameW / 2;
		pos2d.y = FrameH / 2 - pos2d.y;

		tgtPoints.push_back(pos2d);
	}

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
}

void ViewGenerator::getNewMesh()
{
	ASAPSolver asap(FrameW, FrameH, GridX, GridY);
	asap.solve(oriPoints, tgtPoints, this->deformedMesh);

	Mat newMesh(FrameH, FrameW, CV_8UC3, Scalar(255, 255, 255));
	for(int y = 0; y < deformedMesh.rows; y++)
		for(int x = 0; x < deformedMesh.cols; x++)
		{
			Vec2f pos = deformedMesh.at<Vec2f>(y, x);
			circle(newMesh, Point(pos[0], pos[1]), 2, Scalar(0, 0, 0), -1);
		}
	imshow("newMesh", newMesh);
	waitKey(0);

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
		tgtPoints_real.push_back(tgtPoints2D[i]);
		ScenePoints3D.push_back(oriPoints3D[i]);
	}
}

ViewGenerator::~ViewGenerator()
{
}
