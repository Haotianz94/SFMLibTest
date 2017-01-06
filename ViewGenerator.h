#pragma once
#include "ASAPSolver.h"
#include "CameraHelper.h"
#include "Warper.h"

class ViewGenerator
{
public:
	vector<Point2f> oriPoints;
	vector<Point3f> ScenePoints3D;
	vector<Point2f> tgtPoints;
	vector<Point2f> tgtPoints_real;
	Mat_<Vec2f> deformedMesh;  //From original uniform grid
	int GridX, GridY;
	int FrameW; int FrameH;
public:
	void getNewFeaturesPos(CameraModel& newViewCM);
	void getNewMesh();
	ViewGenerator(vector<Point2f>& oriPoints2D, vector<Point2f>& tgtPoints2D, vector<Point3f>& oriPoints3D,
		int FrameWidth, int FrameHeight,
		int GridXNum, int GridYNum);
	ViewGenerator();
	~ViewGenerator();

	Point2f warp(Point2f pos) { return Point2f(pos.x/2+FrameW/4, pos.y/2+FrameH/4); }
};

