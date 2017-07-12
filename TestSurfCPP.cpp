/**
* @file SURF_Homography
* @brief SURF detector + descriptor + FLANN Matcher + FindHomography
* @author A. Huaman
*/

#include <stdio.h>  
#include <iostream>  
#include <cv.h>  
#include "opencv2/core/core.hpp"  
#include <opencv2/opencv.hpp>    
#include "opencv2/features2d/features2d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/nonfree/features2d.hpp"  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/nonfree/nonfree.hpp>    



using namespace cv;
using namespace std;
/*

#ifdef _DEBUG  
#pragma comment (lib, "opencv_calib3d246d.lib")  
#pragma comment (lib, "opencv_contrib246d.lib")  
#pragma comment (lib,"opencv_imgproc246d.lib")  
#pragma comment (lib, "opencv_core246d.lib")  
#pragma comment (lib, "opencv_features2d246d.lib")  
#pragma comment (lib, "opencv_flann246d.lib")  
#pragma comment (lib, "opencv_gpu246d.lib")  
#pragma comment (lib, "opencv_highgui246d.lib")  
#pragma comment (lib, "opencv_legacy246d.lib")  
#pragma comment (lib, "opencv_ml246d.lib")  
#pragma comment (lib, "opencv_objdetect246d.lib")  
#pragma comment (lib, "opencv_ts246d.lib")  
#pragma comment (lib, "opencv_video246d.lib")  
#pragma comment (lib, "opencv_nonfree246d.lib")  
#else  
#pragma comment (lib, "opencv_calib3d246.lib")  
#pragma comment (lib, "opencv_contrib246.lib")  
#pragma comment (lib, "opencv_imgproc246.lib")  
#pragma comment (lib, "opencv_core246.lib")  
#pragma comment (lib, "opencv_features2d246.lib")  
#pragma comment (lib, "opencv_flann246.lib")  
#pragma comment (lib, "opencv_gpu246.lib")  
#pragma comment (lib, "opencv_highgui246.lib")  
#pragma comment (lib, "opencv_legacy246.lib")  
#pragma comment (lib, "opencv_ml246.lib")  
#pragma comment (lib, "opencv_objdetect246.lib")  
#pragma comment (lib, "opencv_ts246.lib")  
#pragma comment (lib, "opencv_video246.lib")  
#pragma comment (lib, "opencv_nonfree246.lib")  
#endif  
*/

int Xmain()
{
	initModule_nonfree();//初始化模块，使用SIFT或SURF时用到   
	Ptr<FeatureDetector> detector = FeatureDetector::create("SURF");//创建SIFT特征检测器，可改成SURF/ORB  
	Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create("SURF");//创建特征向量生成器，可改成SURF/ORB  
	Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");//创建特征匹配器    
	if (detector.empty() || descriptor_extractor.empty())
		cout << "fail to create detector!";

	//读入图像    
	Mat img1 = imread("a1.jpg");
	Mat img2 = imread("a2.jpg");

	//特征点检测    
	double t = getTickCount();//当前滴答数    
	vector<KeyPoint> m_LeftKey, m_RightKey;
	detector->detect(img1, m_LeftKey);//检测img1中的SIFT特征点，存储到m_LeftKey中    
	detector->detect(img2, m_RightKey);
	cout << "图像1特征点个数:" << m_LeftKey.size() << endl;
	cout << "图像2特征点个数:" << m_RightKey.size() << endl;

	//根据特征点计算特征描述子矩阵，即特征向量矩阵    
	Mat descriptors1, descriptors2;
	descriptor_extractor->compute(img1, m_LeftKey, descriptors1);
	descriptor_extractor->compute(img2, m_RightKey, descriptors2);
	t = ((double)getTickCount() - t) / getTickFrequency();
	cout << "SURF算法用时：" << t << "秒" << endl;

	cout << "图像1特征描述矩阵大小：" << descriptors1.size()
		<< "，特征向量个数：" << descriptors1.rows << "，维数：" << descriptors1.cols << endl;
	cout << "图像2特征描述矩阵大小：" << descriptors2.size()
		<< "，特征向量个数：" << descriptors2.rows << "，维数：" << descriptors2.cols << endl;

	//画出特征点    
	Mat img_m_LeftKey, img_m_RightKey;
	drawKeypoints(img1, m_LeftKey, img_m_LeftKey, Scalar::all(-1), 0);
	drawKeypoints(img2, m_RightKey, img_m_RightKey, Scalar::all(-1), 0);
	//imshow("Src1",img_m_LeftKey);    
	//imshow("Src2",img_m_RightKey);    

	//特征匹配    
	vector<DMatch> matches;//匹配结果    
	descriptor_matcher->match(descriptors1, descriptors2, matches);//匹配两个图像的特征矩阵    
	cout << "Match个数：" << matches.size() << endl;

	//计算匹配结果中距离的最大和最小值    
	//距离是指两个特征向量间的欧式距离，表明两个特征的差异，值越小表明两个特征点越接近    
	double max_dist = 0;
	double min_dist = 100;
	for (int i = 0; i<matches.size(); i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	cout << "最大距离：" << max_dist << endl;
	cout << "最小距离：" << min_dist << endl;

	//筛选出较好的匹配点    
	vector<DMatch> goodMatches;
	for (int i = 0; i<matches.size(); i++)
	{
		if (matches[i].distance < 0.25 * max_dist)
		{
			goodMatches.push_back(matches[i]);
		}
	}
	cout << "goodMatch个数：" << goodMatches.size() << endl;

	//画出匹配结果    
	Mat img_matches;
	//红色连接的是匹配的特征点对，绿色是未匹配的特征点    
	drawMatches(img1, m_LeftKey, img2, m_RightKey, goodMatches, img_matches,
		Scalar::all(-1)/*CV_RGB(255,0,0)*/, CV_RGB(0, 255, 0), Mat(), 2);

	imshow("MatchSURF", img_matches);
	IplImage result = img_matches;

	waitKey(0);


/*
	//RANSAC匹配过程  
	vector<DMatch> m_Matches = goodMatches;
	// 分配空间  
	int ptCount = (int)m_Matches.size();
	Mat p1(ptCount, 2, CV_32F);
	Mat p2(ptCount, 2, CV_32F);

	// 把Keypoint转换为Mat  
	Point2f pt;
	for (int i = 0; i<ptCount; i++)
	{
		pt = m_LeftKey[m_Matches[i].queryIdx].pt;
		p1.at<float>(i, 0) = pt.x;
		p1.at<float>(i, 1) = pt.y;

		pt = m_RightKey[m_Matches[i].trainIdx].pt;
		p2.at<float>(i, 0) = pt.x;
		p2.at<float>(i, 1) = pt.y;
	}

	// 用RANSAC方法计算F  
	Mat m_Fundamental;
	vector<uchar> m_RANSACStatus;       // 这个变量用于存储RANSAC后每个点的状态  
	findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC);

	// 计算野点个数  

	int OutlinerCount = 0;
	for (int i = 0; i<ptCount; i++)
	{
		if (m_RANSACStatus[i] == 0)    // 状态为0表示野点  
		{
			OutlinerCount++;
		}
	}
	int InlinerCount = ptCount - OutlinerCount;   // 计算内点  
	cout << "内点数为：" << InlinerCount << endl;


	// 这三个变量用于保存内点和匹配关系  
	vector<Point2f> m_LeftInlier;
	vector<Point2f> m_RightInlier;
	vector<DMatch> m_InlierMatches;

	m_InlierMatches.resize(InlinerCount);
	m_LeftInlier.resize(InlinerCount);
	m_RightInlier.resize(InlinerCount);
	InlinerCount = 0;
	float inlier_minRx = img1.cols;        //用于存储内点中右图最小横坐标，以便后续融合  

	for (int i = 0; i<ptCount; i++)
	{
		if (m_RANSACStatus[i] != 0)
		{
			m_LeftInlier[InlinerCount].x = p1.at<float>(i, 0);
			m_LeftInlier[InlinerCount].y = p1.at<float>(i, 1);
			m_RightInlier[InlinerCount].x = p2.at<float>(i, 0);
			m_RightInlier[InlinerCount].y = p2.at<float>(i, 1);
			m_InlierMatches[InlinerCount].queryIdx = InlinerCount;
			m_InlierMatches[InlinerCount].trainIdx = InlinerCount;

			if (m_RightInlier[InlinerCount].x<inlier_minRx) inlier_minRx = m_RightInlier[InlinerCount].x;   //存储内点中右图最小横坐标  

			InlinerCount++;
		}
	}

	// 把内点转换为drawMatches可以使用的格式  
	vector<KeyPoint> key1(InlinerCount);
	vector<KeyPoint> key2(InlinerCount);
	KeyPoint::convert(m_LeftInlier, key1);
	KeyPoint::convert(m_RightInlier, key2);

	// 显示计算F过后的内点匹配  
	Mat OutImage;
	drawMatches(img1, key1, img2, key2, m_InlierMatches, OutImage);
	cvNamedWindow("Match features", 1);
	cvShowImage("Match features", &IplImage(OutImage));
	waitKey(0);

	cvDestroyAllWindows();

	//矩阵H用以存储RANSAC得到的单应矩阵  
	Mat H = findHomography(m_LeftInlier, m_RightInlier, RANSAC);

	//存储左图四角，及其变换到右图位置  
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = Point(0, 0); obj_corners[1] = Point(img1.cols, 0);
	obj_corners[2] = Point(img1.cols, img1.rows); obj_corners[3] = Point(0, img1.rows);
	std::vector<Point2f> scene_corners(4);
	perspectiveTransform(obj_corners, scene_corners, H);

	//画出变换后图像位置  
	Point2f offset((float)img1.cols, 0);
	line(OutImage, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4);
	line(OutImage, scene_corners[1] + offset, scene_corners[2] + offset, Scalar(0, 255, 0), 4);
	line(OutImage, scene_corners[2] + offset, scene_corners[3] + offset, Scalar(0, 255, 0), 4);
	line(OutImage, scene_corners[3] + offset, scene_corners[0] + offset, Scalar(0, 255, 0), 4);
	imshow("Good Matches & Object detection", OutImage);

	waitKey(0);
	imwrite("warp_position.jpg", OutImage);


	int drift = scene_corners[1].x;                                                        //储存偏移量  

	//新建一个矩阵存储配准后四角的位置  
	int width = int(max(abs(scene_corners[1].x), abs(scene_corners[2].x)));
	int height = img1.rows;                                                                  //或者：int height = int(max(abs(scene_corners[2].y), abs(scene_corners[3].y)));  
	float origin_x = 0, origin_y = 0;
	if (scene_corners[0].x<0) {
		if (scene_corners[3].x<0) origin_x += min(scene_corners[0].x, scene_corners[3].x);
		else origin_x += scene_corners[0].x;
	}
	width -= int(origin_x);
	if (scene_corners[0].y<0) {
		if (scene_corners[1].y) origin_y += min(scene_corners[0].y, scene_corners[1].y);
		else origin_y += scene_corners[0].y;
	}
	//可选：height-=int(origin_y);  
	Mat imageturn = Mat::zeros(width, height, img1.type());

	//获取新的变换矩阵，使图像完整显示  
	for (int i = 0; i<4; i++) { scene_corners[i].x -= origin_x; }     //可选：scene_corners[i].y -= (float)origin_y; }  
	Mat H1 = getPerspectiveTransform(obj_corners, scene_corners);

	//进行图像变换，显示效果  
	warpPerspective(img1, imageturn, H1, Size(width, height));
	imshow("image_Perspective", imageturn);
	waitKey(0);


	//图像融合  
	int width_ol = width - int(inlier_minRx - origin_x);
	int start_x = int(inlier_minRx - origin_x);
	cout << "width: " << width << endl;
	cout << "img1.width: " << img1.cols << endl;
	cout << "start_x: " << start_x << endl;
	cout << "width_ol: " << width_ol << endl;

	uchar* ptr = imageturn.data;
	double alpha = 0, beta = 1;
	for (int row = 0; row<height; row++) {
		ptr = imageturn.data + row*imageturn.step + (start_x)*imageturn.elemSize();
		for (int col = 0; col<width_ol; col++)
		{
			uchar* ptr_c1 = ptr + imageturn.elemSize1();  uchar*  ptr_c2 = ptr_c1 + imageturn.elemSize1();
			uchar* ptr2 = img2.data + row*img2.step + (col + int(inlier_minRx))*img2.elemSize();
			uchar* ptr2_c1 = ptr2 + img2.elemSize1();  uchar* ptr2_c2 = ptr2_c1 + img2.elemSize1();


			alpha = double(col) / double(width_ol); beta = 1 - alpha;

			if (*ptr == 0 && *ptr_c1 == 0 && *ptr_c2 == 0) {
				*ptr = (*ptr2);
				*ptr_c1 = (*ptr2_c1);
				*ptr_c2 = (*ptr2_c2);
			}

			*ptr = (*ptr)*beta + (*ptr2)*alpha;
			*ptr_c1 = (*ptr_c1)*beta + (*ptr2_c1)*alpha;
			*ptr_c2 = (*ptr_c2)*beta + (*ptr2_c2)*alpha;

			ptr += imageturn.elemSize();
		}
	}

	//imshow("image_overlap", imageturn);  
	//waitKey(0);  

	Mat img_result = Mat::zeros(height, width + img2.cols - drift, img1.type());
	uchar* ptr_r = imageturn.data;

	for (int row = 0; row<height; row++) {
		ptr_r = img_result.data + row*img_result.step;

		for (int col = 0; col<imageturn.cols; col++)
		{
			uchar* ptr_rc1 = ptr_r + imageturn.elemSize1();  uchar*  ptr_rc2 = ptr_rc1 + imageturn.elemSize1();

			uchar* ptr = imageturn.data + row*imageturn.step + col*imageturn.elemSize();
			uchar* ptr_c1 = ptr + imageturn.elemSize1();  uchar*  ptr_c2 = ptr_c1 + imageturn.elemSize1();

			*ptr_r = *ptr;
			*ptr_rc1 = *ptr_c1;
			*ptr_rc2 = *ptr_c2;

			ptr_r += img_result.elemSize();
		}

		ptr_r = img_result.data + row*img_result.step + imageturn.cols*img_result.elemSize();
		for (int col = imageturn.cols; col<img_result.cols; col++)
		{
			uchar* ptr_rc1 = ptr_r + imageturn.elemSize1();  uchar*  ptr_rc2 = ptr_rc1 + imageturn.elemSize1();

			uchar* ptr2 = img2.data + row*img2.step + (col - imageturn.cols + drift)*img2.elemSize();
			uchar* ptr2_c1 = ptr2 + img2.elemSize1();  uchar* ptr2_c2 = ptr2_c1 + img2.elemSize1();

			*ptr_r = *ptr2;
			*ptr_rc1 = *ptr2_c1;
			*ptr_rc2 = *ptr2_c2;

			ptr_r += img_result.elemSize();
		}
	}

	imshow("image_result", img_result);
	//imwrite("final_result.jpg",img_result);  */
	return 0;
}