#pragma once

#include "global_header.h"

namespace CVUtil{
	int visualizeMatrix(Mat matrix, Mat& visu, Vec3b colorMin = Vec3b(0,0,0), Vec3b colorMax = Vec3b(255,255,255));

	int visualizeFeatureMatch(Mat & imL, Mat & imR, Mat & imOut, vector<Point2f> &pL, vector<Point2f>& pR, float scale);

	int visualizeMeshAndFeatures(Mat & img, Mat & imgOut, vector<Point2f> & features, Mat_<Vec2f> & mesh_vertices);
}