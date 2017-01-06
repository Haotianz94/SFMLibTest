#include "ASAPSolver.h"


//#include "Features2DFunction.h"

CostDataTermXY::CostDataTermXY( vector<double> & weights, Point2f control_pt )
{
	weight00_ = weights[0];
	weight10_ = weights[1];
	weight01_ = weights[2];
	weight11_ = weights[3];

	x_ = control_pt.x;
	y_ = control_pt.y;
}

CostDataTermX::CostDataTermX( vector<double> & weights, Point2f control_pt )
{
	weight00_ = weights[0], 
	weight10_ = weights[1], 
	weight01_ = weights[2], 
	weight11_ = weights[3];

	x_ = control_pt.x;
	y_ = control_pt.y;
}
CostDataTermY::CostDataTermY( vector<double> & weights, Point2f control_pt )
{
	weight00_ = weights[0], 
	weight10_ = weights[1], 
	weight01_ = weights[2], 
	weight11_ = weights[3];

	x_ = control_pt.x;
	y_ = control_pt.y;
}

ASAPSolver::ASAPSolver( int dimX, int dimY, int gridNumX, int gridNumY )
{
	Mesh m(dimX, dimY, gridNumX, gridNumY);
	m_mesh = m;
	//cout<<1<<endl;
}

ASAPSolver::ASAPSolver( const Mesh & mesh )
{
	m_mesh = mesh;
	
}

void ASAPSolver::solve( vector<Point2f> & pIn, 
	vector<Point2f> & pOut, 
	Mat_<Vec2f> &deformed_mesh, 
	double lambda /*= 1*/ )
{


		int ngx = m_mesh.getGridNumX();
		int ngy = m_mesh.getGridNumY();
		//cout<<ngx<<" "<<ngy<<endl;

		int vertice_num_x = ngx+1;
		int vertice_num_y = ngy+1;

		double * solve_x = new double[vertice_num_x*vertice_num_y];
		double * solve_y = new double[vertice_num_x*vertice_num_y];

		//Init solve values
		FOR(y, 0, ngy){

			FOR(x, 0, ngx){

				solve_x[y*vertice_num_x+x]=(m_mesh.getGrid(x,y).getTL().x);
				solve_y[y*vertice_num_x+x]=(m_mesh.getGrid(x,y).getTL().y);
			
				if(x == ngx-1){

					solve_x[y*vertice_num_x+x+1] = (m_mesh.getGrid(x,y).getDR().x);
					solve_y[y*vertice_num_x+x+1] = (m_mesh.getGrid(x,y).getTL().y);
				
				}
				if(y == ngy-1){
					
					solve_x[(y+1)*vertice_num_x+x] = (m_mesh.getGrid(x,y).getDR().x);
					solve_y[(y+1)*vertice_num_x+x] = (m_mesh.getGrid(x,y).getTL().y);
	
				}
				if(x == ngx-1 && y == ngy-1){
				
					solve_x[(y+1)*vertice_num_x+x+1]=(m_mesh.getGrid(x,y).getDR().x);
					solve_y[(y+1)*vertice_num_x+x+1]=(m_mesh.getGrid(x,y).getDR().y);

				}
			}
		}


		Problem problem;
		//Add DataTerm
		FOR(i, 0, pIn.size()){

			vector<double> weights = m_mesh.getGrid(pIn[i]).getWeights(pIn[i]);
			Point2f control_pt = pOut[i];


			vector<Vec2i> vertice_idx = this->gridIdx2verticesIdx(pIn[i]);


			problem.AddResidualBlock(
				new AutoDiffCostFunction<CostDataTermXY, 2,1,1,1,1,1,1,1,1>
				(new CostDataTermXY(weights, control_pt)), NULL, 
				&solve_x[vertice_idx[0][1]*vertice_num_x+vertice_idx[0][0]],
				&solve_x[vertice_idx[1][1]*vertice_num_x+vertice_idx[1][0]],
				&solve_x[vertice_idx[2][1]*vertice_num_x+vertice_idx[2][0]],
				&solve_x[vertice_idx[3][1]*vertice_num_x+vertice_idx[3][0]],
				&solve_y[vertice_idx[0][1]*vertice_num_x+vertice_idx[0][0]],
				&solve_y[vertice_idx[1][1]*vertice_num_x+vertice_idx[1][0]],
				&solve_y[vertice_idx[2][1]*vertice_num_x+vertice_idx[2][0]],
				&solve_y[vertice_idx[3][1]*vertice_num_x+vertice_idx[3][0]]);


		}

		//Add SmoothTerm
		double ratio = (double)m_mesh.getDIMY()/(double)m_mesh.getGridNumY()/(double)(m_mesh.getDIMX()/(double)m_mesh.getGridNumX());

		FOR(y, 0, ngy){
			FOR(x, 0, ngx){
				

				problem.AddResidualBlock(
					new AutoDiffCostFunction<CostSmoothTermXY, 2,1,1,1,1,1,1>
					(new CostSmoothTermXY(lambda, ratio)), NULL, 
					&solve_x[y*vertice_num_x+x],
					&solve_x[(y+1)*vertice_num_x+x+1],
					&solve_x[(y+1)*vertice_num_x+x],
					&solve_y[y*vertice_num_x+x],
					&solve_y[(y+1)*vertice_num_x+x+1],
					&solve_y[(y+1)*vertice_num_x+x]
				);
				problem.AddResidualBlock(
					new AutoDiffCostFunction<CostSmoothTermXY, 2,1,1,1,1,1,1>
					(new CostSmoothTermXY(lambda, ratio)), NULL, 
					&solve_x[(y+1)*vertice_num_x+x+1],
					&solve_x[y*vertice_num_x+x],
					&solve_x[y*vertice_num_x+x+1],
					&solve_y[(y+1)*vertice_num_x+x+1],
					&solve_y[y*vertice_num_x+x],
					&solve_y[y*vertice_num_x+x+1]
				);
				problem.AddResidualBlock(
					new AutoDiffCostFunction<CostSmoothTermXY, 2,1,1,1,1,1,1>
					(new CostSmoothTermXY(lambda, 1.0/ratio)), NULL, 
					&solve_x[(y+1)*vertice_num_x+x],
					&solve_x[y*vertice_num_x+x+1],
					&solve_x[(y+1)*vertice_num_x+x+1],
					&solve_y[(y+1)*vertice_num_x+x],
					&solve_y[y*vertice_num_x+x+1],
					&solve_y[(y+1)*vertice_num_x+x+1]
				);
				problem.AddResidualBlock(
					new AutoDiffCostFunction<CostSmoothTermXY, 2,1,1,1,1,1,1>
					(new CostSmoothTermXY(lambda, 1.0/ratio)), NULL, 
					&solve_x[y*vertice_num_x+x+1],
					&solve_x[(y+1)*vertice_num_x+x],
					&solve_x[y*vertice_num_x+x],
					&solve_y[y*vertice_num_x+x+1],
					&solve_y[(y+1)*vertice_num_x+x],
					&solve_y[y*vertice_num_x+x]
				);

			}
		}



		//Solve
		Solver::Options options;
		options.max_num_iterations = 50000;
		//options.linear_solver_type = ceres::CGNR;
		options.linear_solver_type = ceres::DENSE_QR;
		//options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;

		Solver::Summary summary;
		Solve(options, &problem, &summary);

		std::cout << summary.BriefReport() << "\n";
		/*double sumPinPout = 0, sumPoutPout = 0;
		FOR(i, 0, pOut.size()){
			vector<double> weight = m_mesh.getGrid(pIn[i]).getWeights(pIn[i]);
			vector<Vec2i> vertice_idx = this->gridIdx2verticesIdx(pIn[i]);
			Point2f V00;
			V00.x = solve_x[vertice_idx[0][1]*vertice_num_x + vertice_idx[0][0]];
			V00.y = solve_y[vertice_idx[0][1]*vertice_num_x + vertice_idx[0][0]];
			Point2f V10;
			V10.x = solve_x[vertice_idx[1][1]*vertice_num_x + vertice_idx[1][0]];
			V10.y = solve_y[vertice_idx[1][1]*vertice_num_x + vertice_idx[1][0]];
			Point2f V01;
			V01.x = solve_x[vertice_idx[2][1]*vertice_num_x + vertice_idx[2][0]];
			V01.y = solve_y[vertice_idx[2][1]*vertice_num_x + vertice_idx[2][0]];
			Point2f V11;
			V11.x = solve_x[vertice_idx[3][1]*vertice_num_x + vertice_idx[3][0]];
			V11.y = solve_y[vertice_idx[3][1]*vertice_num_x + vertice_idx[3][0]];

			
			Point2f po = weight[0]*V00+ weight[1]*V10+weight[2]*V01+weight[3]*V11;
			Point2f pi = weight[0]*m_mesh.getGrid(pIn[i]).getTL()+weight[1]*m_mesh.getGrid(pIn[i]).getDL()\
				+weight[2]*m_mesh.getGrid(pIn[i]).getTR()+weight[3]*m_mesh.getGrid(pIn[i]).getDR();
		

		}*/
		//cout<<sumPinPout<<" "<<sumPoutPout<<endl;

		//Copy result to deformed_mesh
		deformed_mesh = Mat_<Vec2f>::zeros(ngy+1, ngx+1);

		FOR(y, 0, ngy+1){
			FOR(x, 0, ngx+1){
				deformed_mesh[y][x][0] = (float)solve_x[y*vertice_num_x + x];
				deformed_mesh[y][x][1] = (float)solve_y[y*vertice_num_x + x];

			}
		}
		delete []solve_x;
		delete []solve_y;


	return ;

}

vector<Vec2i> ASAPSolver::gridIdx2verticesIdx( Point2f p )
{
	int x, y;
	m_mesh.getGridIdx(p,x,y);

	vector<Vec2i> verticesIdx;
	verticesIdx.push_back(Vec2i(x,y));
	verticesIdx.push_back(Vec2i(x,y+1));
	verticesIdx.push_back(Vec2i(x+1,y));
	verticesIdx.push_back(Vec2i(x+1,y+1));
	return verticesIdx;
}

/*
bool ok(Mat &mask, int x, int y){
	if(x<0||x>=mask.cols||y<0||y>=mask.rows)
		return 0;
	if(mask.at<uchar>(y,x)==255)
		return 1;
	return 0;
}*/
/*
void ASAPSolver::warpBruteForce( Mat & imgIn, Mat_<Vec2f> & deformed_mesh, Mat & imgOut, bool bShowGrid, bool bigger_bord )
{   

	int xBase = 200;
	int yBase = 200;
	if(!bigger_bord){
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
		int newx = xBase+weight[0]*deformed_mesh[yth][xth][0]+weight[1]*deformed_mesh[yth+1][xth][0]\
			+weight[2]*deformed_mesh[yth][xth+1][0]+weight[3]*deformed_mesh[yth+1][xth+1][0];
		
		int newy = yBase+weight[0]*deformed_mesh[yth][xth][1]+weight[1]*deformed_mesh[yth+1][xth][1]\
			+weight[2]*deformed_mesh[yth][xth+1][1]+weight[3]*deformed_mesh[yth+1][xth+1][1];
		/ *cout<<"Ori: "<<x<<" "<<y<<endl;
		FOR(j, 0, 4)
			cout<<weight[j]<<" ";
		cout<<endl;
		cout<<newx<<" "<<newy<<endl;* /
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

		FOR_PIXELS(y,x, deformed_mesh){
			if(x<deformed_mesh.cols-1){
				line(imgOut, Point2f(xBase+deformed_mesh[y][x][0],yBase+deformed_mesh[y][x][1]), Point2f(xBase+deformed_mesh[y][x+1][0],yBase+deformed_mesh[y][x+1][1]), Scalar(255,255,255),3);
			}
			if(y<deformed_mesh.rows-1){
				line(imgOut, Point2f(xBase+deformed_mesh[y][x][0],yBase+deformed_mesh[y][x][1]), Point2f(xBase+deformed_mesh[y+1][x][0],yBase+deformed_mesh[y+1][x][1]), Scalar(255,255,255),3);
			}
			circle(imgOut, Point(xBase+deformed_mesh[y][x][0],yBase+deformed_mesh[y][x][1]),5, Scalar(0,0,255),-1);

		}
		
	}
}*/

void ASAPSolver::estimateBundledCameras(Mat_<Vec2f>& deformed_mesh, vector<vector<Mat>> & bundled_cameras)
{

	int ngx = m_mesh.getGridNumX();
	int ngy = m_mesh.getGridNumY();

	bundled_cameras.resize(ngy);

	FOR(i, 0, ngy)
		bundled_cameras[i].resize(ngx);
	FOR(y, 0, ngy){
		FOR(x, 0, ngx){
			vector<Point2f> srcPts, tarPts;
			srcPts.push_back(m_mesh.getGrid(x,y).getTL());
			srcPts.push_back(m_mesh.getGrid(x,y).getDL());
			srcPts.push_back(m_mesh.getGrid(x,y).getTR());
			srcPts.push_back(m_mesh.getGrid(x,y).getDR());
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
			m.convertTo(bundled_cameras[y][x], CV_32FC1);

			//cout<<bundled_cameras[y][x]<<endl;
		}
	}
}

/*
double ASAPSolver::solve_adaptive( vector<Point2f> & pIn, vector<Point2f> & pOut, Mat_<Vec2f> &deformed_mesh )
{
	double L = 0.3, step = 0.3, terminal = 3.0;
	double minError = 100000;
	double minL = 1000000;
	for(; L<terminal; L+=step){
		cout<<"Iter - Lambda: "<<L<<endl;
		Mat_<Vec2f> d_mesh;
		this->solve(pIn, pOut, d_mesh, L);
		BundledCameras bc(m_mesh, d_mesh);

		double tDataError = 0;
		//calculate Data Term
		FOR(p, 0, pIn.size()){
			int x, y;
			m_mesh.getGridIdx(pIn[p], x, y);
			Mat camera = bc.getCamera(x, y);
			Point2f pT;
			vector<Point2f> pi, po;
			pi.push_back(pIn[p]);
			perspectiveTransform(pi, po, camera);
			pT = po[0] - pOut[p];
			tDataError+= pT.ddot(pT);
		}
		tDataError/= pIn.size();

		double beta = 0.0005, tSmoothError = 0;
		//calculate Smooth Term
		//visit record

		int dir[3][2]={{0,1},{1,0},{1,1}};
		map<pair<int,int>, bool> visit;
		vector<vector<Mat>> cmrs = bc.getAllCameras();
		vector<vector<Mat>> Ncmrs(cmrs.size());
		//Normalize F
		FOR(y, 0, cmrs.size()){
			Ncmrs[y].resize(cmrs[y].size());
			FOR(x, 0, cmrs[y].size()){
				//cout<<cmrs[y][x]<<endl;
				normalize(cmrs[y][x], Ncmrs[y][x], 1);
				//cout<<cmrs[y][x]<<" "<<Ncmrs[y][x]<<endl;
			}
		}
		FOR(y, 0, m_mesh.getGridNumY()){
			FOR(x, 0, m_mesh.getGridNumX()){
				FOR(d, 0, 3){
					int xx = x+dir[d][1];
					int yy = y+dir[d][0];
					if(xx>=0 && xx<m_mesh.getGridNumX() && yy>=0 && yy< m_mesh.getGridNumY()){
							double norm_ =norm(Ncmrs[y][x], Ncmrs[yy][xx], NORM_L2);
							tSmoothError+=norm_*norm_;
					}

				}
			}
		}
		double tError = tDataError + beta* tSmoothError;
		cout<<"Error: "<<tError<<"DataError: "<<tDataError<<" SmoothError: "<<beta*tSmoothError<<endl;
		if(tError< minError){
			minError = tError;
			minL = L;
			deformed_mesh = d_mesh.clone();
		}
	}
	cout<<minL<<endl;
	return minL;
}*/

/*
void ASAPSolver::warpViaBundledCameras( Mat& imgIn, vector<vector<Mat>> & bundled_cameras, Mat & imgOut, bool bShowGrid, bool bigger_bord )
{
	int xBase = 200;
	int yBase = 200;
	if(!bigger_bord){
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
		Mat H = bundled_cameras[yth][xth];
		//cout<<H<<endl;
		vector<Point2f> pIn, pOut;
		pIn.push_back(Point2f(x,y));
		perspectiveTransform(pIn, pOut, H);
		//cout<<pIn<<" "<<pOut<<endl;
		int newx = xBase + pOut[0].x; 
		
		int newy = yBase+ pOut[0].y;

		/ *cout<<"Ori: "<<x<<" "<<y<<endl;
		FOR(j, 0, 4)
			cout<<weight[j]<<" ";
		cout<<endl;
		cout<<newx<<" "<<newy<<endl;* /
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
				perspectiveTransform(pIn, pOut, bundled_cameras[y][x]);
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
				perspectiveTransform(pIn, pOut, bundled_cameras[y][x]);
				circle(imgOut, pOut[0]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);
				circle(imgOut, pOut[1]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);
				circle(imgOut, pOut[2]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);	
				circle(imgOut, pOut[3]+Point2f(xBase, yBase), 5, Scalar(0,0,255), -1);	
			}
		}
		
	}
}*/

