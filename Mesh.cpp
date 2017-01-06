#include "Mesh.h"

Mesh::Mesh()
{
	m_vertices.clear();
}

Mesh::Mesh( float dimX, float dimY, int GridNumX /*= 16*/, int GridNumY /*= 16*/ )
{
	m_dimX = dimX;
	m_dimY = dimY;
	m_GridNumX = GridNumX;
	m_GridNumY = GridNumY;
	m_GridLenX = dimX/GridNumX;
	m_GridLenY = dimY/GridNumY;
	FOR(y, 0, GridNumX){
		FOR(x, 0, GridNumY){
			m_vertices.push_back(Grid( x*m_GridLenX,
									(x+1)*m_GridLenX, 
									y*m_GridLenY, 
									(y+1)*m_GridLenY) );
		}
	}
}

Mesh::Mesh( const Mesh & rhs )
{
	m_GridLenX = rhs.m_GridLenX;
	m_GridLenY = rhs.m_GridLenY;
	m_dimX = rhs.m_dimX;
	m_dimY = rhs.m_dimY;
	m_GridNumX = rhs.m_GridNumX;
	m_GridNumY = rhs.m_GridNumY;
	m_vertices = rhs.m_vertices;
}

Mesh & Mesh::operator=( const Mesh & rhs )
{
	m_GridLenX = rhs.m_GridLenX;
	m_GridLenY = rhs.m_GridLenY;
	m_dimX = rhs.m_dimX;
	m_dimY = rhs.m_dimY;
	m_GridNumX = rhs.m_GridNumX;
	m_GridNumY = rhs.m_GridNumY;
	m_vertices = rhs.m_vertices;
	return *this;
}

Grid & Mesh::getGrid( int xth, int yth )
{
	int idx = yth*m_GridNumX+xth;
	if(idx>=m_GridNumX*m_GridNumY){
		cout<<xth<<" "<<yth<<endl;
		cout<<"GridSize: "<<m_GridLenX<<" "<<m_GridLenY<<endl;
		WM_ERROR("Error: Max Grid Index Reached.");
	}
	return m_vertices[idx];
}

Grid & Mesh::getGrid( Point2f v )
{
	int xth = v.x/m_GridLenX;
	int yth = v.y/m_GridLenY;
	return getGrid(xth, yth);
}



Mesh::~Mesh()
{

}

void Mesh::getGridIdx( Point2f v, int & xth, int &yth )
{
	xth = floor(v.x/m_GridLenX);
	yth = floor(v.y/m_GridLenY);
}

void Mesh::cvtVertices2MatVec2f( Mat_<Vec2f>& m )
{
	int ngx = m_GridNumX;
	int ngy = m_GridNumY;
	m=Mat_<Vec2f>::zeros(ngy+1, ngx+1);
	FOR(y, 0, ngy){

		FOR(x, 0, ngx){

			m[y][x][0]=(this->getGrid(x,y).getTL().x);
			m[y][x][1]=(this->getGrid(x,y).getTL().y);

			if(x == ngx-1){

				m[y][x+1][0]=(this->getGrid(x,y).getDR().x);
				m[y][x+1][1]=(this->getGrid(x,y).getTL().y);
				//vertices_to_solve[y][x+1]=(p2);

			}
			if(y == ngy-1){

				m[y+1][x][0]=(this->getGrid(x,y).getTL().x);
				m[y+1][x][1]=(this->getGrid(x,y).getDR().y);
				//vertices_to_solve[y+1][x]=(p2);
			}
			if(x == ngx-1 && y == ngy-1){

				m[y+1][x+1][0]=(this->getGrid(x,y).getDR().x);
				m[y+1][x+1][1]=(this->getGrid(x,y).getDR().y);
				//vertices_to_solve[y+1][x+1]=(p2);
			}
		}
	}
}

void Mesh::visualize( Mat & image )
{
	int ngx = getGridNumX();
	int ngy = getGridNumY();
	float xBase = 0, yBase = 0;
	FOR(y, 0, ngy){
		FOR(x, 0, ngx){
			vector<Point2f> pIn;
			pIn.push_back(getGrid(x, y).getTL());
			pIn.push_back(getGrid(x, y).getTR());
			pIn.push_back(getGrid(x, y).getDL());
			pIn.push_back(getGrid(x, y).getDR());
			line(image, pIn[0]+Point2f(xBase, yBase), pIn[1]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
			line(image, pIn[0]+Point2f(xBase, yBase), pIn[2]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
			line(image, pIn[2]+Point2f(xBase, yBase), pIn[3]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
			line(image, pIn[3]+Point2f(xBase, yBase), pIn[1]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
		}
	}
	FOR(y, 0, ngy){
		FOR(x, 0, ngx){
			vector<Point2f> pIn;
			pIn.push_back(getGrid(x, y).getTL());
			pIn.push_back(getGrid(x, y).getTR());
			pIn.push_back(getGrid(x, y).getDL());
			pIn.push_back(getGrid(x, y).getDR());
			//perspectiveTransform(pIn, pOut, bundled_cameras.getCamera(x,y));
			circle(image, pIn[0]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);
			circle(image, pIn[1]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);
			circle(image, pIn[2]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);	
			circle(image, pIn[3]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);	
		}
	}
}


