#include "CV_Util2.h"



int CVUtil::visualizeMatrix( Mat matrix, Mat & visu, Vec3b colorMin /*= Vec3b(0,0,0)*/, Vec3b colorMax /*= Vec3b(255,255,255)*/ )
{
	vector<Mat> bgrChannels;

	Mat tmp;
	normalize(matrix, tmp, 0, 1, NORM_MINMAX);

	FOR(i, 0, 3){

		bgrChannels.push_back(matrix.clone());

		bgrChannels[i] = (colorMin[i] + tmp * (colorMax[i] - colorMin[i]));
		
	}

	merge(bgrChannels, visu);

	visu.convertTo(visu, CV_8UC3);
	return 1;
}

int CVUtil::visualizeFeatureMatch( Mat & imL, Mat & imR, Mat & imOut, vector<Point2f> &pL, vector<Point2f>& pR, float scale )
{
	Mat imL_, imR_;
	resize(imL, imL_,Size(),scale,scale);
	resize(imR, imR_,Size(),scale,scale);
	int R = imL_.rows;
	int C = imL_.cols*2;
	imOut = Mat::zeros(R,C,CV_8UC3);
	Rect r1(Point(0,0),Point(imL_.cols, imL_.rows));
	Rect r2(Point(imL_.cols, 0), Point(2*imL_.cols, imL_.rows));
	imL_.copyTo(imOut(r1));
	imR_.copyTo(imOut(r2));
	FOR(i, 0, pL.size()){
		int x1 = pL[i].x*scale;
		int y1 = pL[i].y*scale;
		int x2 = pR[i].x*scale;
		int y2 = pR[i].y*scale;
		Scalar color(rand()%255, rand()%255, rand()%255);
		line(imOut,Point(x1,y1),Point(imL_.cols+x2,y2), color);
	}
	return 1;
}

int CVUtil::visualizeMeshAndFeatures( Mat & img, Mat & imgOut, vector<Point2f> & features, Mat_<Vec2f> & mesh_vertices )
{
	imgOut = img.clone();
	FOR_PIXELS(y,x,mesh_vertices){
		if(x<mesh_vertices.cols-1){
			line(imgOut, Point2f(mesh_vertices[y][x][0],mesh_vertices[y][x][1]), Point2f(mesh_vertices[y][x+1][0],mesh_vertices[y][x+1][1]), Scalar(255,255,255),2, CV_AA);
		}
		if(y<mesh_vertices.rows-1){
			line(imgOut, Point2f(mesh_vertices[y][x][0],mesh_vertices[y][x][1]), Point2f(mesh_vertices[y+1][x][0],mesh_vertices[y+1][x][1]), Scalar(255,255,255),2, CV_AA);
		}
		circle(imgOut, Point(mesh_vertices[y][x][0],mesh_vertices[y][x][1]),3, Scalar(0,0,255), -1,CV_AA);

	}
	FOR(i, 0, features.size()){
		circle(imgOut, features[i],3, Scalar(0,255,0),-1,CV_AA);
	}
	return 1;
}
