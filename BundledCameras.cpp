#include "BundledCameras.h"

#include "fstream"

BundledCameras::BundledCameras( int nx /*= 16*/, int ny /*= 16 */):m_camera_num_x(nx), m_camera_num_y(ny)
{
	m_cameras.resize(ny);
	Mat h = Mat::eye(3,3,CV_64FC1);
	FOR(y, 0, ny){
		m_cameras[y].resize(nx);
		FOR(x, 0, nx){
			m_cameras[y][x] = h.clone();
		}
	}
}

BundledCameras::BundledCameras( const BundledCameras & rhs )
{
	m_camera_num_x = rhs.m_camera_num_x;
	m_camera_num_y = rhs.m_camera_num_y;
	//m_cameras = rhs.m_cameras;
	m_cameras.resize(rhs.m_cameras.size());
	FOR(y, 0, rhs.m_cameras.size()){
		m_cameras[y].resize(rhs.m_cameras[y].size());
		FOR(x, 0, rhs.m_cameras[y].size()){
			m_cameras[y][x] = rhs.m_cameras[y][x].clone();
			//cout<<m_cameras[y][x]<<endl;
		}
	}
}

BundledCameras & BundledCameras::operator=( const BundledCameras & rhs )
{
	m_camera_num_x = rhs.m_camera_num_x;
	m_camera_num_y = rhs.m_camera_num_y;
	m_cameras.resize(rhs.m_cameras.size());
	FOR(y, 0, rhs.m_cameras.size()){
		m_cameras[y].resize(rhs.m_cameras[y].size());
		FOR(x, 0, rhs.m_cameras[y].size()){
			m_cameras[y][x] = rhs.m_cameras[y][x].clone();
		//	cout<<m_cameras[y][x]<<endl;
		}
	}
	return * this;
}

BundledCameras::BundledCameras( vector<vector<Mat>> & bundled_cameras )
{
	m_camera_num_y = bundled_cameras.size();
	m_camera_num_x = bundled_cameras[0].size();

	m_cameras.resize(m_camera_num_y);
	FOR(y, 0, bundled_cameras.size()){
		m_cameras[y].resize(m_camera_num_x);
		FOR(x, 0, bundled_cameras[y].size())
			m_cameras[y][x] = bundled_cameras[y][x].clone();
	}
	//m_cameras = bundled_cameras;
/*
	FOR(y, 0, bundled_cameras.size()){
		FOR(x, 0, bundled_cameras[y].size())
			m_cameras[y].push_back(bundled_cameras[y][x].clone());
	}*/
}

BundledCameras::BundledCameras( Mesh & mesh, Mat_<Vec2f> & deformed_mesh )
{
	m_camera_num_x = mesh.getGridNumX();
	m_camera_num_y = mesh.getGridNumY();

	m_cameras.resize(m_camera_num_y);

	FOR(i, 0, m_camera_num_y)
		m_cameras[i].resize(m_camera_num_x);
	FOR(y, 0, m_camera_num_y){
		FOR(x, 0, m_camera_num_x){
			vector<Point2f> srcPts, tarPts;
			srcPts.push_back(mesh.getGrid(x,y).getTL());
			srcPts.push_back(mesh.getGrid(x,y).getDL());
			srcPts.push_back(mesh.getGrid(x,y).getTR());
			srcPts.push_back(mesh.getGrid(x,y).getDR());
			tarPts.push_back(Point2f(deformed_mesh[y][x][0],deformed_mesh[y][x][1]));
			tarPts.push_back(Point2f(deformed_mesh[y+1][x][0],deformed_mesh[y+1][x][1]));
			tarPts.push_back(Point2f(deformed_mesh[y][x+1][0],deformed_mesh[y][x+1][1]));
			tarPts.push_back(Point2f(deformed_mesh[y+1][x+1][0],deformed_mesh[y+1][x+1][1]));
		/*	FOR(i, 0, 4){
				cout<<srcPts[i]<<" ";
			}
			cout<<endl;
			FOR(i, 0, 4){
				cout<<tarPts[i]<<" ";
			}
			cout<<endl;*/
			//vector<int> good_idx;
			Mat m = getPerspectiveTransform(srcPts, tarPts);
	
			m.convertTo(m_cameras[y][x], CV_64FC1);
			//cout<<m_cameras[y][x]<<endl;

			//cout<<bundled_cameras[y][x]<<endl;
		}
	}
}

BundledCameras::~BundledCameras()
{
	//m_cameras.clear();
}



void BundledCameras::setCamera( int xth, int yth, Mat & c )
{
	m_cameras[yth][xth] = c.clone();
}

void BundledCameras::saveCameras( string file_name )
{
	int ny = m_camera_num_y;
	int nx = m_camera_num_x;
	//cout<<nx<<" "<<ny<<endl;

	ofstream f(file_name, ios::binary|ios::trunc);
	if(!f.is_open())
		return ;

	f.write((char *) & ny, sizeof(int));
	f.write((char *) & nx, sizeof(int));
	
	int sz = sizeof(double)*3*3;
	FOR(ty, 0, ny){
		FOR(tx, 0, nx){
/*
			cout<<m_cameras[ty][tx].rows<<" "<<m_cameras[ty][tx].cols<<endl;
			cout<<m_cameras[ty][tx]<<endl;
			cout<<m_cameras[ty][tx].type()<<endl;*/
		
			
			f.write((char *) m_cameras[ty][tx].data, sz);
			//cout<<m_cameras[ty][tx]<<endl;
		}
	}

	f.close();
	return ;

}

void BundledCameras::loadCameras( string file_name )
{
	ifstream f(file_name, ios::binary);

	if(!f.is_open())
		return ;

	int ny, nx;
	f.read( (char*) & ny, sizeof(int));
	f.read( (char*) & nx, sizeof(int));
	//cout<<ny<<" "<<nx<<endl;
	m_camera_num_y = ny;
	m_camera_num_x = nx;

	m_cameras.resize(ny);
	Mat h = Mat::eye(3,3,CV_64FC1);
	FOR(y, 0, ny){
		m_cameras[y].resize(nx);
		FOR(x, 0, nx){
			m_cameras[y][x] = h.clone();
		}
	}
	int sz = sizeof(double)*3*3;
	FOR(ty, 0, ny){
		FOR(tx, 0, nx){
			f.read((char *) m_cameras[ty][tx].data, sz);
			//cout<<m_cameras[ty][tx]<<endl;
		}
	}
	return ;
}

void BundledCameras::init( Mesh & mesh, Mat_<Vec2f> & deformed_mesh )
{
	m_camera_num_x = mesh.getGridNumX();
	m_camera_num_y = mesh.getGridNumY();

	m_cameras.resize(m_camera_num_y);

	FOR(i, 0, m_camera_num_y)
		m_cameras[i].resize(m_camera_num_x);
	FOR(y, 0, m_camera_num_y){
		FOR(x, 0, m_camera_num_x){
			vector<Point2f> srcPts, tarPts;
			srcPts.push_back(mesh.getGrid(x,y).getTL());
			srcPts.push_back(mesh.getGrid(x,y).getDL());
			srcPts.push_back(mesh.getGrid(x,y).getTR());
			srcPts.push_back(mesh.getGrid(x,y).getDR());
			tarPts.push_back(Point2f(deformed_mesh[y][x][0],deformed_mesh[y][x][1]));
			tarPts.push_back(Point2f(deformed_mesh[y+1][x][0],deformed_mesh[y+1][x][1]));
			tarPts.push_back(Point2f(deformed_mesh[y][x+1][0],deformed_mesh[y][x+1][1]));
			tarPts.push_back(Point2f(deformed_mesh[y+1][x+1][0],deformed_mesh[y+1][x+1][1]));
		/*	FOR(i, 0, 4){
				cout<<srcPts[i]<<" ";
			}
			cout<<endl;
			FOR(i, 0, 4){
				cout<<tarPts[i]<<" ";
			}
			cout<<endl;*/
			//vector<int> good_idx;
			Mat m = getPerspectiveTransform(srcPts, tarPts);
			m.convertTo(m_cameras[y][x], CV_64FC1);

			//cout<<bundled_cameras[y][x]<<endl;
		}
	}
}
