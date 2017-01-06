#include "Warper.h"
#include "CV_Util.h"
Warper::Warper( Mesh & mesh )
{
	m_mesh = mesh;
}
void Warper::loadMesh( Mesh & mesh )
{
	m_mesh = mesh;
}


bool ok(Mat &mask, int x, int y){
	if(x<0||x>=mask.cols||y<0||y>=mask.rows)
		return 0;
	if(mask.at<uchar>(y,x)==255)
		return 1;
	return 0;
}
void Warper::warpBilateralInterpolate( Mat &imgIn, 
	Mat_<Vec2f> & deformed_mesh_vertices, 
	Mat & imgOut, 
	bool bShowGrid /*= 0*/, 
	bool bBiggerBord /*= 0*/ )
{
	int xBase = 200;
	int yBase = 200;
	if(!bBiggerBord){
		xBase = 0;
		yBase = 0;
	}
	imgOut.create(2*yBase+imgIn.rows, 2*xBase+imgIn.cols,CV_8UC3);
	imgOut.setTo(Vec3b(255,255,255));
	Mat checkMaskOut = Mat::zeros(RC(imgOut),CV_8UC1);
	FOR_PIXELS(y,x,imgIn){
		vector<double> weight = m_mesh.getGrid(Point2f(x,y)).getWeights(Point2f(x,y));
		int xth, yth;
		m_mesh.getGridIdx(Point2f(x,y), xth, yth);
		int newx = xBase+weight[0]*deformed_mesh_vertices[yth][xth][0]+weight[1]*deformed_mesh_vertices[yth+1][xth][0]\
			+weight[2]*deformed_mesh_vertices[yth][xth+1][0]+weight[3]*deformed_mesh_vertices[yth+1][xth+1][0];
		
		int newy = yBase+weight[0]*deformed_mesh_vertices[yth][xth][1]+weight[1]*deformed_mesh_vertices[yth+1][xth][1]\
			+weight[2]*deformed_mesh_vertices[yth][xth+1][1]+weight[3]*deformed_mesh_vertices[yth+1][xth+1][1];
		/*cout<<"Ori: "<<x<<" "<<y<<endl;
		FOR(j, 0, 4)
			cout<<weight[j]<<" ";
		cout<<endl;
		cout<<newx<<" "<<newy<<endl;*/
		if(newy>=0&& newy<imgOut.rows && newx>=0 && newx<imgOut.cols){
			imgOut.at<Vec3b>(newy, newx) = imgIn.at<Vec3b>(y,x);
			checkMaskOut.at<uchar>(newy, newx) = 255;
		}
	}
	int dir[8][2]={-1,-1, -1,0, -1,1, 0,-1, 0, 1, 1, -1, 1, 0, 1, 1};
	FOR_PIXELS(y,x,imgOut){
		if(checkMaskOut.at<uchar>(y,x) == 0){
			FOR(j, 0, 8){
				int xx = x+dir[j][0];
				int yy = y+dir[j][1];
				if (ok(checkMaskOut, xx, yy)){
					imgOut.at<Vec3b>(y,x)=imgOut.at<Vec3b>(yy,xx);
					break;
				}
			}
		}
	}
	if(bShowGrid){

		FOR_PIXELS(y,x, deformed_mesh_vertices){
			if(x<deformed_mesh_vertices.cols-1){
				line(imgOut, Point2f(xBase+deformed_mesh_vertices[y][x][0],yBase+deformed_mesh_vertices[y][x][1]), Point2f(xBase+deformed_mesh_vertices[y][x+1][0],yBase+deformed_mesh_vertices[y][x+1][1]), Scalar(255,255,255),3);
			}
			if(y<deformed_mesh_vertices.rows-1){
				line(imgOut, Point2f(xBase+deformed_mesh_vertices[y][x][0],yBase+deformed_mesh_vertices[y][x][1]), Point2f(xBase+deformed_mesh_vertices[y+1][x][0],yBase+deformed_mesh_vertices[y+1][x][1]), Scalar(255,255,255),3);
			}
			circle(imgOut, Point(xBase+deformed_mesh_vertices[y][x][0],yBase+deformed_mesh_vertices[y][x][1]),5, Scalar(0,0,255),-1);

		}
		
	}
}

void Warper::warpBundledCameras( Mat & imgIn, BundledCameras & bundled_cameras, Mat & imgOut, bool bShowGrid /*= 0*/, bool bBiggerBord /*= 0*/ )
{
	int xBase = 200;
	int yBase = 200;
	if(!bBiggerBord){
		xBase = 0;
		yBase = 0;
	}
	imgOut.create(2*yBase+imgIn.rows, 2*xBase+imgIn.cols,CV_8UC3);
	imgOut.setTo(Vec3b(255,255,255));
	Mat checkMaskOut = Mat::zeros(RC(imgOut),CV_8UC1);
	FOR_PIXELS(y,x,imgIn){
		vector<double> weight = m_mesh.getGrid(Point2f(x,y)).getWeights(Point2f(x,y));
		int xth, yth;
		m_mesh.getGridIdx(Point2f(x,y), xth, yth);
		Mat H = bundled_cameras.getCamera(xth,yth);
		//cout<<H<<endl;
		vector<Point2f> pIn, pOut;
		pIn.push_back(Point2f(x,y));
		perspectiveTransform(pIn, pOut, H);
		//cout<<pIn<<" "<<pOut<<endl;
		int newx = xBase + pOut[0].x; 
		
		int newy = yBase+ pOut[0].y;

		/*cout<<"Ori: "<<x<<" "<<y<<endl;
		FOR(j, 0, 4)
			cout<<weight[j]<<" ";
		cout<<endl;
		cout<<newx<<" "<<newy<<endl;*/
		if(newy>=0&& newy<imgOut.rows && newx>=0 && newx<imgOut.cols){
			imgOut.at<Vec3b>(newy, newx) = imgIn.at<Vec3b>(y,x);
			checkMaskOut.at<uchar>(newy, newx) = 255;
		}
	}
	int dir[8][2]={-1,-1, -1,0, -1,1, 0,-1, 0, 1, 1, -1, 1, 0, 1, 1};
	FOR_PIXELS(y,x,imgOut){
		if(checkMaskOut.at<uchar>(y,x) == 0){
			FOR(j, 0, 8){
				int xx = x+dir[j][0];
				int yy = y+dir[j][1];
				if (ok(checkMaskOut, xx, yy)){
					imgOut.at<Vec3b>(y,x)=imgOut.at<Vec3b>(yy,xx);
					break;
				}
			}
		}
	}
	if(bShowGrid){

		int ngx = m_mesh.getGridNumX();
		int ngy = m_mesh.getGridNumY();
		
		FOR(y, 0, ngy){
			FOR(x, 0, ngx){
				vector<Point2f> pIn, pOut;
				pIn.push_back(m_mesh.getGrid(x, y).getTL());
				pIn.push_back(m_mesh.getGrid(x, y).getTR());
				pIn.push_back(m_mesh.getGrid(x, y).getDL());
				pIn.push_back(m_mesh.getGrid(x, y).getDR());
				perspectiveTransform(pIn, pOut, bundled_cameras.getCamera(x,y));
				line(imgOut,pOut[0]+Point2f(xBase, yBase), pOut[1]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
				line(imgOut, pOut[0]+Point2f(xBase, yBase), pOut[2]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
				line(imgOut, pOut[2]+Point2f(xBase, yBase), pOut[3]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
				line(imgOut, pOut[3]+Point2f(xBase, yBase), pOut[1]+Point2f(xBase, yBase),Scalar(255,255,255), 3);
			}
		}
		FOR(y, 0, ngy){
			FOR(x, 0, ngx){
				vector<Point2f> pIn, pOut;
				pIn.push_back(m_mesh.getGrid(x, y).getTL());
				pIn.push_back(m_mesh.getGrid(x, y).getTR());
				pIn.push_back(m_mesh.getGrid(x, y).getDL());
				pIn.push_back(m_mesh.getGrid(x, y).getDR());
				perspectiveTransform(pIn, pOut, bundled_cameras.getCamera(x,y));
				circle(imgOut, pOut[0]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);
				circle(imgOut, pOut[1]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);
				circle(imgOut, pOut[2]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);	
				circle(imgOut, pOut[3]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);	
			}
		}
		
	}
}

void Warper::warpWithRemap( Mat & imgIn, 
	Mat_<Vec2f> & deformed_mesh_vertices, 
	Mat & imgOut, 
	bool bShowGrid /*= 0*/, bool bBiggerBord /*= 0*/ )
{

	int xBase = 200;
	int yBase = 200;
	if(!bBiggerBord){
		xBase = 0;
		yBase = 0;
	}
/*
	vector<Point2f> features;
	Mat imgO;
	CVUtil::visualizeMeshAndFeatures(imgIn, imgO, features, deformed_mesh_vertices);
	SHOW_IMG(imgO);*/
	
	imgOut.create(2*yBase+imgIn.rows, 2*xBase+imgIn.cols,CV_8UC3);
	imgOut.setTo(Vec3b(255,255,255));


	cv::Mat map_x(imgIn.rows+2*xBase,imgIn.cols+2*xBase,CV_32FC1,cv::Scalar(-1));
	cv::Mat map_y(imgIn.rows+2*xBase,imgIn.cols+2*yBase,CV_32FC1,cv::Scalar(-1));

	//vector<vector<Mat>> homos(m_mesh.getGridNumY());

	FOR(y, 0, m_mesh.getGridNumY()){
		//homos[y].resize(m_mesh.getGridNumX());
		FOR(x, 0, m_mesh.getGridNumX()){

			vector<Point2f> pBefore, pAfter;
			pBefore.push_back(m_mesh.getGrid(x,y).getTL());
			pBefore.push_back(m_mesh.getGrid(x,y).getDL());
			pBefore.push_back(m_mesh.getGrid(x,y).getTR());
			pBefore.push_back(m_mesh.getGrid(x,y).getDR());
			pAfter.push_back(Point2f(xBase, yBase)+Point2f(deformed_mesh_vertices[y][x][0],deformed_mesh_vertices[y][x][1]));
			pAfter.push_back(Point2f(xBase, yBase)+Point2f(deformed_mesh_vertices[y+1][x][0],deformed_mesh_vertices[y+1][x][1]));
			pAfter.push_back(Point2f(xBase, yBase)+Point2f(deformed_mesh_vertices[y][x+1][0],deformed_mesh_vertices[y][x+1][1]));
			pAfter.push_back(Point2f(xBase, yBase)+Point2f(deformed_mesh_vertices[y+1][x+1][0],deformed_mesh_vertices[y+1][x+1][1]));
			Mat homos = findHomography(pAfter, pBefore);

/*
			FOR(t, 0, pAfter.size())
				cout<<pAfter[t]<<" ";
			cout<<endl;*/

			double ox1 = m_mesh.getGrid(x,y).getTL().x;
			double oy1 = m_mesh.getGrid(x,y).getTL().y;
			double ox2 = m_mesh.getGrid(x,y).getDR().x;
			double oy2 = m_mesh.getGrid(x,y).getDR().y;

			double x1 = deformed_mesh_vertices[y][x][0];
			double x2 = deformed_mesh_vertices[y+1][x][0];
			double x3 = deformed_mesh_vertices[y][x+1][0];
			double x4 = deformed_mesh_vertices[y+1][x+1][0];

			double y1 = deformed_mesh_vertices[y][x][1];
			double y2 = deformed_mesh_vertices[y+1][x][1];
			double y3 = deformed_mesh_vertices[y][x+1][1];
			double y4 = deformed_mesh_vertices[y+1][x+1][1];

			double minx = x1<x2? x1:x2;minx = minx < x3? minx :x3;minx = minx < x4? minx :x4;
			double maxx = x1>x2? x1:x2;maxx = maxx > x3? maxx :x3;maxx = maxx > x4? maxx :x4;

			double miny = y1<y2? y1:y2;miny = miny < y3? miny :y3;miny = miny < y4? miny :y4;
			double maxy = y1>y2? y1:y2;maxy = maxy > y3? maxy :y3;maxy = maxy > y4? maxy :y4;
			
			minx+=xBase;
			maxx+=xBase;
			miny+=yBase;
			maxy+=yBase;
			
			vector<Point2f> pIn, pOut;
			for(int ii = floor(miny);ii<=ceil(maxy);ii++)
			{
				for(int jj = floor(minx);jj<=ceil(maxx);jj++)
				{
					
					if(ii >= 0 && ii< map_x.rows && jj >= 0 && jj < map_x.cols)
						pIn.push_back(Point2f(jj,ii));
					
				}
			}
/*
			cout<<y<<" "<<x<<endl;
			cout<<homos<<endl;	
			cout<<"pIn: "<<pIn.size()<<endl;*/
			if(pIn.size()>0)
			{
				perspectiveTransform(pIn, pOut, homos);

				FOR(p, 0, pOut.size()){
					double ux = pOut[p].x;
					double uy = pOut[p].y;
					if(ux >= ox1 && ux < ox2 && uy >= oy1 && uy < oy2)
					{
						map_x.at<float>(Point2i(pIn[p])) = ux;
						map_y.at<float>(Point2i(pIn[p])) = uy;
					}
				}
			}

		}
	}

	cv::remap(imgIn, imgOut, map_x, map_y, cv::INTER_CUBIC);

	if(bShowGrid){

		FOR_PIXELS(y,x, deformed_mesh_vertices){
			if(x<deformed_mesh_vertices.cols-1){
				line(imgOut, Point2f(xBase+deformed_mesh_vertices[y][x][0],yBase+deformed_mesh_vertices[y][x][1]), Point2f(xBase+deformed_mesh_vertices[y][x+1][0],yBase+deformed_mesh_vertices[y][x+1][1]), Scalar(255,255,255),3);
			}
			if(y<deformed_mesh_vertices.rows-1){
				line(imgOut, Point2f(xBase+deformed_mesh_vertices[y][x][0],yBase+deformed_mesh_vertices[y][x][1]), Point2f(xBase+deformed_mesh_vertices[y+1][x][0],yBase+deformed_mesh_vertices[y+1][x][1]), Scalar(255,255,255),3);
			}
			circle(imgOut, Point(xBase+deformed_mesh_vertices[y][x][0],yBase+deformed_mesh_vertices[y][x][1]),5, Scalar(0,0,255),-1);

		}

	}
}

void Warper::warpBundledCamerasX( Mat & imgIn, 
	BundledCameras & bundled_cameras, 
	Mat & imgOut, 
	bool bShowGrid /*= 0*/, 
	bool bBiggerBord /*= 0*/ )
{

	Mat_<Vec3f> ave_vtx(m_mesh.getGridNumY()+1, m_mesh.getGridNumX()+1, Vec3f(0,0,0));
	Mat_<Vec3f> ave_vty(m_mesh.getGridNumY()+1, m_mesh.getGridNumX()+1, Vec3f(0,0,0));
	FOR(y, 0, m_mesh.getGridNumY()){
       FOR(x, 0, m_mesh.getGridNumX()){

            cv::Mat B = bundled_cameras.getCamera(x, y);

            double minx = m_mesh.getGrid(x,y).getTL().x;
            double miny = m_mesh.getGrid(x,y).getTL().y;
            double maxx = m_mesh.getGrid(x,y).getDR().x;
            double maxy = m_mesh.getGrid(x,y).getDR().y;

            /**
                0  1
                3  2
                */
            double d0x,d0y,d1x,d1y,d2x,d2y,d3x,d3y;
			vector<Point2f> pIn, pOut;
			pIn.push_back(Point2f(minx, miny));
			pIn.push_back(Point2f(maxx, miny));
			pIn.push_back(Point2f(maxx, maxy));
			pIn.push_back(Point2f(minx, maxy));

			perspectiveTransform(pIn, pOut, B);
/*
            TransformByHomo(B,minx,miny,d0x,d0y);
            TransformByHomo(B,maxx,miny,d1x,d1y);
            TransformByHomo(B,maxx,maxy,d2x,d2y);
            TransformByHomo(B,minx,maxy,d3x,d3y);*/
			d0x = pOut[0].x;
			d0y = pOut[0].y;
			d1x = pOut[1].x;
			d1y = pOut[1].y;
			d2x = pOut[2].x;
			d2y = pOut[2].y;
			d3x = pOut[3].x;
			d3y = pOut[3].y;

            ave_vtx[y][x][0] = ave_vtx[y][x][0] + d0x;
            ave_vtx[y][x][1] = ave_vtx[y][x][1] + d0y;
            ave_vtx[y][x][2] = ave_vtx[y][x][2] + 1;

            ave_vtx[y][x+1][0] = ave_vtx[y][x+1][0] + d1x;
            ave_vtx[y][x+1][1] = ave_vtx[y][x+1][1] + d1y;
            ave_vtx[y][x+1][2] = ave_vtx[y][x+1][2] + 1;

            ave_vtx[y+1][x+1][0] = ave_vtx[y+1][x+1][0] + d2x;
            ave_vtx[y+1][x+1][1] = ave_vtx[y+1][x+1][1] + d2y;
            ave_vtx[y+1][x+1][2] = ave_vtx[y+1][x+1][2] + 1;

            ave_vtx[y+1][x][0] = ave_vtx[y+1][x][0] + d3x;
            ave_vtx[y+1][x][1] = ave_vtx[y+1][x][1] + d3y;
            ave_vtx[y+1][x][2] = ave_vtx[y+1][x][2] + 1;
        }
    }

	Mat_<Vec2f> deformed_mesh(m_mesh.getGridNumY()+1, m_mesh.getGridNumX()+1);
	FOR(y, 0, deformed_mesh.rows){
		FOR(x, 0, deformed_mesh.cols){
            deformed_mesh[y][x][0] = ave_vtx[y][x][0]/ave_vtx[y][x][2];
            deformed_mesh[y][x][1] = ave_vtx[y][x][1]/ave_vtx[y][x][2];
        }
    }
	//cout<<deformed_mesh<<endl;
	this->warpWithRemap(imgIn, deformed_mesh, imgOut, bShowGrid, bBiggerBord);
}



