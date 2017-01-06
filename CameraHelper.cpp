#include "CameraHelper.h"

void CameraHelper::changeCoordinate(cv::Point2f& pos, cv::Point2f& center)
{
	pos.x = pos.x - center.x;
	pos.y = pos.y - center.y;
}

/*

struct DMatch
{
	//三个构造函数  
	DMatch() : queryIdx(-1), trainIdx(-1), imgIdx(-1), distance(std::numeric_limits<float>::max()) {}
	DMatch(int  _queryIdx, int  _trainIdx, float  _distance) :
		queryIdx(_queryIdx), trainIdx(_trainIdx), imgIdx(-1), distance(_distance) {}
	DMatch(int  _queryIdx, int  _trainIdx, int  _imgIdx, float  _distance) :
		queryIdx(_queryIdx), trainIdx(_trainIdx), imgIdx(_imgIdx), distance(_distance) {}

	intqueryIdx;  //此匹配对应的查询图像的特征描述子索引  
	inttrainIdx;   //此匹配对应的训练(模板)图像的特征描述子索引  
	intimgIdx;    //训练图像的索引(若有多个)  
	float distance;  //两个特征向量之间的欧氏距离，越小表明匹配度越高。  
	booloperator < (const DMatch &m) const;
};*/

void CameraHelper::getSIFTforeground(Mat& img1, Mat& img2, Mat& mask1, Mat& mask2)
{
	initModule_nonfree();//初始化模块，使用SIFT或SURF时用到  
	Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");//创建SIFT特征检测器  
	Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create("SIFT");//创建特征向量生成器  
	Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");//创建特征匹配器  
	if (detector.empty() || descriptor_extractor.empty())
		cout << "fail to create detector!";

	//特征点检测  
	double t = getTickCount();//当前滴答数  
	vector<KeyPoint> keypoints1, keypoints2;
	detector->detect(img1, keypoints1, mask1);//检测img1中的SIFT特征点，存储到keypoints1中  
	detector->detect(img2, keypoints2, mask2);
	cout << "图像1特征点个数:" << keypoints1.size() << endl;
	cout << "图像2特征点个数:" << keypoints2.size() << endl;

	//根据特征点计算特征描述子矩阵，即特征向量矩阵  
	Mat descriptors1, descriptors2;
	descriptor_extractor->compute(img1, keypoints1, descriptors1);
	descriptor_extractor->compute(img2, keypoints2, descriptors2);
	t = ((double)getTickCount() - t) / getTickFrequency();
	cout << "SIFT算法用时：" << t << "秒" << endl;
	//画出特征点  
	Mat img_keypoints1, img_keypoints2;
	drawKeypoints(img1, keypoints1, img_keypoints1, Scalar::all(-1), 0);
	drawKeypoints(img2, keypoints2, img_keypoints2, Scalar::all(-1), 0);

	//特征匹配  
	vector<DMatch> matches;//匹配结果  
	descriptor_matcher->match(descriptors1, descriptors2, matches);//匹配两个图像的特征矩阵  
	cout << "Match个数：" << matches.size() << endl;

	//计算匹配结果中距离的最大和最小值  
	//距离是指两个特征向量间的欧式距离，表明两个特征的差异，值越小表明两个特征点越接近  
	double max_dist = 0;
	double min_dist = 100;
	for (int i = 0; i < matches.size(); i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	cout << "最大距离：" << max_dist << endl;
	cout << "最小距离：" << min_dist << endl;

	//筛选出较好的匹配点  
	vector<DMatch> goodMatches;
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < 0.31 * max_dist)
		{
			goodMatches.push_back(matches[i]);
		}
	}
	cout << "goodMatch个数：" << goodMatches.size() << endl;

	//画出匹配结果  
	Mat img_matches;
	//红色连接的是匹配的特征点对，绿色是未匹配的特征点  
	drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches,
		Scalar::all(-1)/*CV_RGB(255,0,0)*/, CV_RGB(0, 255, 0), Mat(), 2);

	imshow("MatchSIFT", img_matches);
	waitKey(0);
}

void CameraHelper::test(cv::Mat& rotMat, cv::Mat& translateMat, double f)
{
	cv::Mat p3d, temp3d;
	p3d.create(cv::Size(1, 3), CV_64F);
	p3d.ptr<double>(0)[0] = -5.75373220444;
	p3d.ptr<double>(1)[0] = -0.00709797209129;
	p3d.ptr<double>(2)[0] = 0.96617192037;
	//cv::multiply(rotMat, p3d, temp3d);
	temp3d = rotMat * p3d;
	cv::add(temp3d, translateMat, p3d);
	double tempa = f / p3d.ptr<double>(2)[0];
	p3d = -tempa * p3d;
	cout << p3d.ptr<double>(0)[0] << ", " << p3d.ptr<double>(1)[0] << "," << p3d.ptr<double>(2)[0] / (-tempa) << endl;
}

void CameraHelper::testTri(BundlerFileLoader& bfl)
{

	for (int i = 0; i < 10; i+=1)
	{
		cout << i << endl;
		OneFeatureInWholeScene& oneFeat = bfl.allFeats[i];
		for (int k = 0; k < oneFeat.numOfVisibelCam - 2; k+=2)
		{
			triangulation(bfl.allCameras[oneFeat.featInAllViews[k].cameraIndex],
				bfl.allCameras[oneFeat.featInAllViews[k + 1].cameraIndex],
				oneFeat.featInAllViews[k].position2D,
				oneFeat.featInAllViews[k + 1].position2D);

			triangulation(bfl.allCameras[oneFeat.featInAllViews[k].cameraIndex],
				bfl.allCameras[oneFeat.featInAllViews[k + 2].cameraIndex],
				oneFeat.featInAllViews[k].position2D,
				oneFeat.featInAllViews[k + 2].position2D);
		}

	}
}

cv::Point3f CameraHelper::triangulation(CameraModel& cam1, CameraModel& cam2, cv::Point2f& p1, cv::Point2f& p2)
{
	//get the 
	Matx34d P1, P2;
	Point3d u1, u2;
	double fMat1[3], fMat2[3];
	fMat1[0] = cam1.focallength;
	fMat1[1] = cam1.focallength;
	fMat1[2] = 1;
	fMat2[0] = cam2.focallength;
	fMat2[1] = cam2.focallength;
	fMat2[2] = 1;
	for (int r = 0; r < 3; r++)
	{
		P1(r, 3) = fMat1[r]*cam1.translation.ptr<double>(r)[0];
		P2(r, 3) = fMat2[r] * cam2.translation.ptr<double>(r)[0];
		for (int c = 0; c < 3; c++)
		{
			P1(r, c) = fMat1[r] * cam1.rotation.ptr<double>(r)[c];
			P2(r, c) = fMat2[r] * cam2.rotation.ptr<double>(r)[c];
		}
	}
	
	u1.x = -p1.x;
	u1.y = -p1.y;
	u1.z = 1;
	u2.x = -p2.x;
	u2.y = -p2.y;
	u2.z = 1;
	cv::Mat x = LinearLSTriangulation(u1, P1, u2, P2);
	cout << x.ptr<double>(0)[0] << ", " << x.ptr<double>(1)[0] << ", " << x.ptr<double>(2)[0] << endl;
	cv::Point3f res;
	res.x = x.ptr<double>(0)[0];
	res.y = x.ptr<double>(1)[0];
	res.z = x.ptr<double>(2)[0];
	return res;
}

cv::Point2f CameraHelper::get2DViewPosition(CameraModel& cam, cv::Point3f& Pos3D)
{
	cv::Mat p3d, temp3d;
	cv::Mat& rotMat = cam.rotation;
	cv::Mat& translateMat = cam.translation;
	double f = cam.focallength;
	p3d.create(cv::Size(1, 3), CV_64F);
	p3d.ptr<double>(0)[0] = Pos3D.x;
	p3d.ptr<double>(1)[0] = Pos3D.y;
	p3d.ptr<double>(2)[0] = Pos3D.z;
	//cv::multiply(rotMat, p3d, temp3d);
	temp3d = rotMat * p3d;
	cv::add(temp3d, translateMat, p3d);
	double tempa = f / p3d.ptr<double>(2)[0];
	p3d = -tempa * p3d;
//	cout << p3d.ptr<double>(0)[0] << ", " << p3d.ptr<double>(1)[0] << "," << p3d.ptr<double>(2)[0] / (-tempa) << endl;
	cv::Point2f p2d;
	p2d.x = p3d.ptr<double>(0)[0];
	p2d.y = p3d.ptr<double>(1)[0];
	return p2d;
}

Mat_<double> CameraHelper::LinearLSTriangulation(
	Point3d u,//homogenous image point (u,v,1)  
	Matx34d P,//camera 1 matrix  
	Point3d u1,//homogenous image point in 2nd camera  
	Matx34d P1//camera 2 matrix  
	)
{
	//build A matrix  
	Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
		);
	//build B vector  
	Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));
	//solve for X  
	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);
	return X;
}