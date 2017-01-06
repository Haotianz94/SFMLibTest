#ifndef WARPER_H_
#define WARPER_H_

#include "mesh.h"
#include "BundledCameras.h"

class Warper{
public:
	Warper(){};
	Warper(Mesh & mesh);
	
	void loadMesh(Mesh & mesh);
	
	void warpBilateralInterpolate(Mat &imgIn, 
		Mat_<Vec2f> & deformed_mesh_vertices, 
		Mat & imOut, 
		bool bShowGrid = 0, 
		bool bBiggerBord = 0);

	void warpBundledCameras(Mat & imgIn,
		BundledCameras & bundled_cameras,
		Mat & imOut,
		bool bShowGrid = 0,
		bool bBiggerBord = 0);

	void warpWithRemap(Mat & imgIn, Mat_<Vec2f> & deformed_mesh_vertices, Mat & imOut,
		bool bShowGrid = 0, bool bBiggerBord = 0);

	void warpBundledCamerasX(Mat & imgIn,
		BundledCameras & bundled_cameras,
		Mat & imOut,
		bool bShowGrid = 0,
		bool bBiggerBord = 0);


private:
	Mesh m_mesh;
};



#endif