#include "global_header.h"

#ifndef BUNDLED_CAMERAS_H_
#define BUNDLED_CAMERAS_H_

#include "Mesh.h"

//Bunlded Camera Class
//Store N*M Homographies
// 
// 
// 

class BundledCameras{
public:
	BundledCameras(int nx = 16, int ny = 16);
	~BundledCameras();
	BundledCameras(const BundledCameras & rhs);
	BundledCameras(vector<vector<Mat>> & bundled_cameras);
	BundledCameras(Mesh & mesh, Mat_<Vec2f> & deformed_mesh);
	BundledCameras & operator=(const BundledCameras & rhs);

	void init(Mesh & mesh, Mat_<Vec2f> & deformed_mesh);
	inline const vector<vector<Mat>> & getAllCameras(){return m_cameras;}
	inline const Mat & getCamera(int xth, int yth){return m_cameras[yth][xth];}
	void setCamera(int xth, int yth, Mat & c);
	inline int getCameraNumX(){return m_camera_num_x;}
	inline int getCameraNumY(){return m_camera_num_y;}
	inline void getCameraIdx(int i, int &x, int &y){
		x = i%m_camera_num_x;
		y = i/m_camera_num_y;
	}
	inline int getVecIdx(int x, int y){
		return y*m_camera_num_x+x;
	}

	void saveCameras(string file_name);
	void loadCameras(string file_name);
	vector<vector<Mat>> m_cameras;	
private:
	int m_camera_num_x;
	int m_camera_num_y;

};



#endif