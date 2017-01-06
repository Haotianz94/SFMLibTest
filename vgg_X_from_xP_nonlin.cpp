#include "vgg_X_from_xP_nonlin.h"

/* Conversion from quaternion and translation to euler angles and camera center */
CvMat* q2rot(CvMat* q_nion)
{
	/* Required variables */
	CvMat*  p_rot_mat;
	double* p_rot_mat_data;
	double* p_qion_data;
	double  xx, xy, xz;
	double  xw, yy, yz;
	double  yw, zz, zw;
	double  q_w, q_x, q_y, q_z;

	/* Create the rot matrix to be returned as output */
	p_rot_mat = cvCreateMat(3, 3, CV_64FC1);
    
	/* Pointers to ease access */
	p_rot_mat_data = p_rot_mat->data.db;
	p_qion_data    = q_nion->data.db;

	/* Initialize quaternion parameters */
	q_w = *(p_qion_data);
	q_x = *(p_qion_data + 1);
	q_y = *(p_qion_data + 2);
	q_z = *(p_qion_data + 3);

	/* First, convert quaternion q(w, x, y, z) to rotation matrix */
    xx = q_x * q_x;
	xy = q_x * q_y;
    xz = q_x * q_z; 
	xw = q_x * q_w; 
	yy = q_y * q_y; 
	yz = q_y * q_z; 
	yw = q_y * q_w; 
	zz = q_z * q_z; 
	zw = q_z * q_w;

	/* Evaluate the parameters of rotation matrix */
	p_rot_mat_data[0] = 1 - 2 * (yy + zz); 
	p_rot_mat_data[3] = 2 * (xy + zw); 
	p_rot_mat_data[6] = 2 * (xz - yw); 
	p_rot_mat_data[1] = 2 * (xy - zw); 
	p_rot_mat_data[4] = 1 - 2 * (xx + zz); 
	p_rot_mat_data[7] = 2 * (yz + xw); 
	p_rot_mat_data[2] = 2 * (xz + yw); 
	p_rot_mat_data[5] = 2 * (yz - xw); 
	p_rot_mat_data[8] = 1 - 2 * (xx + yy);

	return p_rot_mat;
}

CvMat* vgg_X_from_xP_nonlin(CvMat* u1, CvMat** P1, CvMat* imsize1, int K)
{

	CvMat* u;
	CvMat** P=new CvMat* [K];
	CvMat* imsize;
	
	u=cvCreateMat(u1->rows,u1->cols,u1->type);
	cvCopy(u1,u);

	int kp;
	for(kp=0;kp<K;kp++)
	{
		P[kp]=cvCreateMat(P1[kp]->rows,P1[kp]->cols,P1[kp]->type);
		cvCopy(P1[kp],P[kp]);
	}

	imsize=cvCreateMat(imsize1->rows,imsize1->cols,imsize1->type);
	cvCopy(imsize1,imsize);


	CvMat* X;
	CvMat* H;
	CvMat* u_2_rows;
	CvMat* W;
	CvMat* U;
	CvMat* T;
	CvMat* Y;
	CvMat** Q=new CvMat*[K];//Q is a variable not well defined
	
	CvMat* J;
	CvMat* e;
	CvMat* J_tr;
	CvMat* eprev;
	CvMat* JtJ;
	CvMat* Je;
	CvMat* Y_new;

	CvMat* T_2_cols;
	CvMat* T_rest_cols;
	CvMat* X_T;
	CvScalar f, inf;
	int i, mat_id;
	int u_rows = u->rows;
	int u_cols = u->cols;
	double lambda_min, lambda_max;
	CvMat* imsize_col = cvCreateMat(2, 1, CV_64FC1);

	/* K is the number of images */
	if(K < 2)
	{
		printf("\n Cannot reconstruct 3D from 1 image");
		return 0;
	}

	/* Create temporary matrix for the linear function */
	u_2_rows = cvCreateMat(2, u_cols, CV_64FC1);

	/* Initialize the temporary matrix by extracting the first two rows */
	u_2_rows = cvGetRows(u, u_2_rows, 0, 2, 1);

	/* Call the linear function */
	X = vgg_X_from_xP_lin(u_2_rows, P, K, imsize);

	imsize_col = cvGetCol(imsize, imsize_col, 0);
	f = cvSum(imsize_col);
	f.val[0] = 4 / f.val[0];

	/* Create and initialize H matrix */
	H = cvCreateMat(3, 3, CV_64FC1);

	H->data.db[0] = f.val[0];
	H->data.db[1] = 0;
	H->data.db[2] = ((-1) * f.val[0] * cvmGet(imsize, 0, 0)) / 2;
	H->data.db[3] = 0;
	H->data.db[4] = f.val[0];
	H->data.db[5] = ((-1) * f.val[0] * cvmGet(imsize, 1, 0)) / 2;
	H->data.db[6] = 0;
	H->data.db[7] = 0;
	H->data.db[8] = 1;

	for(mat_id = 0; mat_id < K ; mat_id++)
	{
		cvMatMul(H, P[mat_id], P[mat_id]);
	}
	/* u = H * u; */
	cvMatMul(H, u, u);
	/*
	//debug
	printf("....H\n");
	CvMat_printdb(stdout,"%7.3f ",H);
	//debug
	printf("....u\n");
	CvMat_printdb(stdout,"%7.3f ",u);
	*/

	/* Parametrize X such that X = T*[Y;1]; thus x = P*T*[Y;1] = Q*[Y;1] */
	/* Create the SVD matrices X = U*W*V'*/

	X_T = cvCreateMat(X->cols, X->rows, CV_64FC1); /* M * N */
	W = cvCreateMat(X_T->rows, X_T->cols, CV_64FC1); /* M * N */
	U = cvCreateMat(X_T->rows, X_T->rows, CV_64FC1); /* M * M */
	T = cvCreateMat(X_T->cols, X_T->cols, CV_64FC1); /* N * N */
	cvTranspose(X, X_T);
	cvSVD(X_T, W, U, T);


	cvReleaseMat(&W);
	
	/* T = T(:,[2:end 1]); */
	/* Initialize the temporary matrix by extracting the first two columns */
	/* Create temporary matrix for the linear function */
	T_2_cols    = cvCreateMat(T->rows, 2, CV_64FC1);
	T_rest_cols = cvCreateMat(T->rows, (T->cols - 2), CV_64FC1);

	/* Initialize the temporary matrix by extracting the first two columns */
	T_2_cols= sfmGetCols(T,0,0);
	T_rest_cols=sfmGetCols(T,1,T->cols-1);

	T=sfmAlignMatH(T_rest_cols,T_2_cols);

	for(mat_id = 0; mat_id < K ; mat_id++)
	{
	    /* Create temporary matrix for the linear function */
	    Q[mat_id] = cvCreateMat(P[mat_id]->rows, T->cols, CV_64FC1);
		cvMatMul(P[mat_id], T, Q[mat_id]);
	}

	/*
	//debug
	printf("....Q0\n");
	CvMat_printdb(stdout,"%7.3f ",Q[0]);

	//debug
	printf("....Q1\n");
	CvMat_printdb(stdout,"%7.3f ",Q[1]);
*/
	/* Newton estimation */
	/* Create the required Y matrix for the Newton process */
	Y = cvCreateMat(3, 1, CV_64FC1);
	cvSetZero(Y); /* Y = [0;0;0] */

	/* Initialize the infinite array */
	inf.val[0] = INF;
	inf.val[1] = INF;
	inf.val[2] = INF;
	inf.val[3] = INF;
	eprev = cvCreateMat(1, 1, CV_64FC1); 
	cvSet(eprev, inf, 0);

	for(i = 0 ; i < 10 ; i++)
	{
	//	printf("i=%d....\n",i);
		int pass;
		double RCondVal;

		//initialize e,J before using.

		e=cvCreateMat(2*K,1,CV_64FC1);
		J=cvCreateMat(2*K,3,CV_64FC1);

		pass = resid(Y, u, Q, K, e, J);

		J_tr = cvCreateMat(J->cols, J->rows, CV_64FC1);
		cvTranspose(J, J_tr);

		JtJ = cvCreateMat(J->cols, J->cols, CV_64FC1);
		cvMatMul(J_tr, J, JtJ);

		//prevent memory leak;
		cvReleaseMat(&W);

		/* Create the SVD matrices JtJ = U*W*V'*/
		W = cvCreateMat(J->cols, J->cols, CV_64FC1); 

		cvSVD(JtJ, W); 
		/*
		//debug
		printf("....W\n");
		CvMat_printdb(stdout,"%7.3f ",W);
*/
		lambda_max = W->data.db[0];
		lambda_min = W->data.db[((W->rows * W->cols) - 1)];
		RCondVal   = lambda_min / lambda_max;

		if(1 - (cvNorm(e, 0, CV_L2, 0) / cvNorm(eprev, 0, CV_L2, 0)) < 1000 * EPS)
		{
			cvReleaseMat(&J);
			cvReleaseMat(&e);
			cvReleaseMat(&J_tr);
			cvReleaseMat(&JtJ);
			cvReleaseMat(&W);
			break;
		}
		if(RCondVal < 10 * EPS)
		{
			cvReleaseMat(&J);
			cvReleaseMat(&e);
			cvReleaseMat(&J_tr);
			cvReleaseMat(&JtJ);
			cvReleaseMat(&W);
			break;
		}

		
		cvReleaseMat(&eprev);
		eprev = cvCreateMat(e->rows, e->cols, CV_64FC1); 
		cvCopy(e, eprev);

		Je = cvCreateMat(J->cols, e->cols, CV_64FC1);
		cvMatMul(J_tr, e, Je); /* (J'*e) */

		/*
		//debug
		printf("....J_tr\n");
		CvMat_printdb(stdout,"%7.3f ",J_tr);

		//debug
		printf("....e\n");
		CvMat_printdb(stdout,"%7.3f ",e);

		//debug
		printf("....JtJ\n");
		CvMat_printdb(stdout,"%7.3f ",JtJ);
		//debug
		printf("....Je\n");
		CvMat_printdb(stdout,"%7.3f ",Je);

		
		//debug
		printf("....JtJ\n");
		CvMat_printdb(stdout,"%7.3f ",JtJ);
*/
		cvInvert(JtJ,JtJ);
		/* (J'*J)\(J'*e) */
		Je=sfmMatMul(JtJ, Je);
/*
		//debug
		printf("....Je\n");
		CvMat_printdb(stdout,"%7.3f ",Je);
*/
		/* Y = Y - (J'*J)\(J'*e) */
		cvSub(Y, Je, Y, 0);
		/*
		//debug
		printf("....Y\n");
		CvMat_printdb(stdout,"%7.3f ",Y);
		*/
		cvReleaseMat(&J);
		cvReleaseMat(&e);
		cvReleaseMat(&J_tr);
		cvReleaseMat(&JtJ);
		cvReleaseMat(&Je);
		cvReleaseMat(&W);

	}
	Y_new  = cvCreateMat(4, 1, CV_64FC1);
	PutMatV(Y,Y_new,0);
	Y_new->data.db[3]=1;


	/*
	//debug
	printf("....Y_new\n");
	CvMat_printdb(stdout,"%7.3f ",Y_new);
	printf("....T\n");
	CvMat_printdb(stdout,"%7.3f ",T);
*/

	/* Obtain the new X */
    cvMatMul(T, Y_new, X);


	cvReleaseMat(&H);
	cvReleaseMat(&u_2_rows);
	
	cvReleaseMat(&U);
	cvReleaseMat(&T);
	cvReleaseMat(&Y);


	cvReleaseMat(&Y_new);
	cvReleaseMat(&T_2_cols);
	cvReleaseMat(&T_rest_cols);

	for(kp=0;kp<K;kp++)
	{
		cvReleaseMat(&P[kp]);
	}
	cvReleaseMat(&u);
	cvReleaseMat(&imsize);
	cvReleaseMatGrp(Q,K);

	return X;
}

int resid(CvMat* Y, CvMat* u, CvMat** Q, int num_Q_mats, CvMat* e, CvMat* J)
{
	int i, mat_id;
	CvMat* q;
	CvMat* x0;
	CvMat* x;
	CvMat* temp_mat1;
	CvMat* temp_mat2;
	CvMat* temp_mat3_1;
	CvMat* temp_mat3_2;

	/* Create the initial e and J matrices */
//	e = cvCreateMat(0, 0, CV_64FC1);
//	J = cvCreateMat(0, 0, CV_64FC1);

	for(mat_id = 0 ; mat_id < num_Q_mats ; mat_id++)
	{

		/* Copy data into the new matrix */
		q  = sfmGetCols(Q[mat_id], 0, 2);
		/* Copy data to form the x0 matrix */
		x0 = sfmGetCols(Q[mat_id],3);

		/* x = q*Y + x0 */
		x=sfmMatMul(q, Y);
		cvAdd(x, x0, x);
		/*
		//debug
		printf("....x\n");
		CvMat_printdb(stdout,"%7.3f ",x);
*/
		/* Temporary matrix to aid evaluations of e */
		temp_mat1 = cvCreateMat(2, 1, CV_64FC1);
		temp_mat2 = cvCreateMat(2, 1, CV_64FC1);

		/* Evaluate and save temporary data */
		temp_mat1->data.db[0] = x->data.db[0] / x->data.db[2];
		temp_mat1->data.db[1] = x->data.db[1] / x->data.db[2];

		temp_mat2->data.db[0] = u->data.db[0*u->cols + mat_id];
		temp_mat2->data.db[1] = u->data.db[1*u->cols + mat_id];

		/* e = [e; x(1:2)/x(3)-u(1:2,k)] */
		cvSub(temp_mat1,temp_mat2,temp_mat1);
		PutMatV(temp_mat1,e,mat_id*2);

		/*
		//debug
		printf("....e\n");
		CvMat_printdb(stdout,"%7.3f ",e);
*/
		/* Temporary matrix to aid evaluations of Y */
		cvReleaseMat(&temp_mat2);
		cvReleaseMat(&temp_mat1);
		//memory leak
		temp_mat1   = cvCreateMat(1, 3, CV_64FC1);
		temp_mat2   = cvCreateMat(1, 3, CV_64FC1);
		temp_mat3_1 = cvCreateMat(1, 3, CV_64FC1);
		temp_mat3_2 = cvCreateMat(1, 3, CV_64FC1);

		for(i = 0 ; i < 3 ; i++)
		{
			/* Copy data into the temp matrices */
			temp_mat1->data.db[i] = (q->data.db[i] * x->data.db[2]) / 
				                    (x->data.db[2] * x->data.db[2]); /* x(3)*q(1,:) / x(3)^2 */

			temp_mat2->data.db[i] = (q->data.db[i + 3] * x->data.db[2]) / 
				                    (x->data.db[2] * x->data.db[2]); /* x(3)*q(2,:) / x(3)^2 */

			temp_mat3_1->data.db[i] = (q->data.db[i + 6] * x->data.db[0]) / 
				                      (x->data.db[2] * x->data.db[2]); /* x(1)*q(3,:) / x(3)^2 */

			temp_mat3_2->data.db[i] = (q->data.db[i + 6] * x->data.db[1]) / 
				                      (x->data.db[2] * x->data.db[2]); /* x(2)*q(3,:) / x(3)^2 */
		}
		/*
		//for debug
		printf("....temp_mat1\n");
		CvMat_printdb(stdout,"%7.3f ",temp_mat1);
		printf("....temp_mat2\n");
		CvMat_printdb(stdout,"%7.3f ",temp_mat2);

		printf("....temp_mat3_1\n");
		CvMat_printdb(stdout,"%7.3f ",temp_mat3_1);
		printf("....temp_mat3_2\n");
		CvMat_printdb(stdout,"%7.3f ",temp_mat3_2);
*/

		/* x(3)*q(1,:)-x(1)*q(3,:) */
		cvSub(temp_mat1, temp_mat3_1, temp_mat1);

		/* x(3)*q(2,:)-x(2)*q(3,:) */
		cvSub(temp_mat2, temp_mat3_2, temp_mat2);

		/* [x(3)*q(1,:)-x(1)*q(3,:) x(3)*q(2,:)-x(2)*q(3,:)]/x(3)^2 */
		temp_mat1 = cat(temp_mat1, temp_mat2, 1);		
		
		/* J = [J; [x(3)*q(1,:)-x(1)*q(3,:) x(3)*q(2,:)-x(2)*q(3,:)]/x(3)^2] */
		PutMatV(temp_mat1,J,mat_id*2);
		/*
		//for debug.
		printf("....J\n");
		CvMat_printdb(stdout,"%7.3f ",J);
*/
		cvReleaseMat(&temp_mat3_2);
		cvReleaseMat(&temp_mat3_1);
		cvReleaseMat(&temp_mat2);
		cvReleaseMat(&temp_mat1);
		cvReleaseMat(&x);
		cvReleaseMat(&x0);
		cvReleaseMat(&q);
	}
	return 1;
}

/* Concatenate two matrices */
/* dim: 1 = Vertical concatenate, 2 = Horizontal concatenate */
CvMat* cat(CvMat* matA, CvMat* matB, int dim)
{
	CvMat* p_cat_mat;
	int row_A = matA->rows;
	int col_A = matA->cols;
	int row_B = matB->rows;
	int col_B = matB->cols;
	int i, j;

	/* Vertical concatenation */
	if (dim == 1)
	{
		if(col_A != col_B)
		{
			printf("Arguments dimensions are not consistent");
			exit(0);
		}
		else
		{
			/* create the required new matrix */
			p_cat_mat = cvCreateMat((row_A + row_B), col_A, CV_64FC1);

			/* Copy the data from the two matrices and concatenate vertically */
			for(i = 0; i < (row_A + row_B) ; i++)
			{
				for(j = 0 ; j < col_A ; j++)
				{
					if(i < row_A)
						p_cat_mat->data.db[(i * col_A) + j] = matA->data.db[(i * col_A) + j];
					else
						p_cat_mat->data.db[(i * col_B) + j] = matB->data.db[((i - row_A) * col_B) + j];
				}
			}
		}
	}
	/* Horizontal concatenation */
	else if (dim == 2)
	{
		if(row_A != row_B)
		{
			printf("Arguments dimensions are not consistent");
			return NULL;
		}
		else
		{
			/* create the required new matrix */
			p_cat_mat = cvCreateMat(row_A, (col_A + col_B), CV_64FC1);

			/* Copy data from the two matrices and concatenate horizontally */
			for(i = 0; i < row_A ; i++)
			{
				for(j = 0 ; j < (col_A + col_B) ; j++)
				{
					if(j < col_A)
						p_cat_mat->data.db[(i * (col_A + col_B)) + j] = matA->data.db[(i * col_A) + j];
					else
						p_cat_mat->data.db[(i * (col_A + col_B)) + j] = matB->data.db[(i * col_B) + (j - col_A)];
				}
			}
		}
	}
	else
	{
		printf("This dimension is not supported\n");
		return NULL;
	}
	return p_cat_mat;
}


/* Find the row-wise or column wise mean of the matrix */
/* mode: 1 = column-wise, 2 = row_wise */
CvMat* mean(CvMat* mat, int mode)
{
	CvMat* p_mean_mat;
	int rows = mat->rows;
	int cols = mat->cols;
	int i, j;
	double row_sum = 0, col_sum = 0;

	/* Column-wise mean */
	if(mode == 1)
	{
		/* create the required new matrix */
		p_mean_mat = cvCreateMat(1, cols, CV_64FC1);

		/* Evaluate and save the mean value of each column */
		for(i = 0 ; i < cols ; i++)
		{
			col_sum = 0;
			for(j = 0 ; j < rows ; j++)
			{
				col_sum += mat->data.db[i + (j * cols)];
			}
			p_mean_mat->data.db[i] = col_sum / rows;
		}
	}
	/* Row-wise mean */
	else if(mode == 2)
	{
		/* create the required new matrix */
		p_mean_mat = cvCreateMat(rows, 1, CV_64FC1);

		/* Evaluate and save the mean value of each row */
		for(i = 0 ; i < rows ; i++)
		{
			row_sum = 0;
			for(j = 0 ; j < cols ; j++)
			{
				row_sum += mat->data.db[(i * cols) + j];
			}
			p_mean_mat->data.db[i] = row_sum / cols;
		}
	}
	else
	{
		printf("This mode is not valid\n");
		return NULL;
	}
	return p_mean_mat;
}