#include "Scene3DRecover.h"

#include "Geometry/Include/EuclideanGeometry.h"
#include "Geometry/Include/SE3Geometry.h"
#include "Geometry/Include/GrassmannGeometry.h"
#include "Geometry/Include/EssentialGeometry.h"
#include "Geometry/Include/DTIAffineInvariantGeometry.h"
#include "MeanShiftLib/Include/VectorPointSet.h"
#include "MeanShiftLib/Include/EssentialPointSet.h"
#include "MeanShiftLib/Include/MeanShift.h"

using namespace std;
void Scene3DRecover::getForeGround3DAllFrames()
{
	for (int i = 0; i < matchedFramesID.size(); i++)
	{
		int id1 = matchedFramesID[i].first;
		int idInSFM1 = frame2Cam[make_pair(1, id1)];
		//In the bundler file, the imgs are re-ordered according to the camera parameters
		int id2 = matchedFramesID[i].second;
		int idInSFM2 = frame2Cam[make_pair(2, id2)];
		vector<scenePointOnPair> ScnPoints;
		this->recoverFore3D1F(ScnPoints, cam1ImgNames[id1], cam2ImgNames[id2],
			sfmLoader.allCameras[idInSFM1], sfmLoader.allCameras[idInSFM2],
			allSiftsCam1[id1], allSiftsCam2[id2]);
		
		allForeGroundScenePoints.push_back(ScnPoints);
		break;//*************
	}
}

void Scene3DRecover::getBackGround3DAllFrames()
{
	for(int i = 0; i < cam1Num; i++)
		allBackGroundPointsCam1.push_back(vector<scenePoint>());
	for(int i = 0; i < cam2Num; i++)
		allBackGroundPointsCam2.push_back(vector<scenePoint>());
	
		for(auto& feat : sfmLoader.allFeats)
			for(auto& featOneView : feat.featInAllViews)
				{
					scenePoint sp;
					sp.pos2D.x = featOneView.position2D.x + FrameW/2;
					sp.pos2D.y = FrameH/2 - featOneView.position2D.y; 
					sp.scenePos = feat.position3D;
					int camId = cam2Frame[featOneView.cameraIndex].first;
					int frameId = cam2Frame[featOneView.cameraIndex].second;
					//REPORT(camId);
					//REPORT(frameId);
					if(camId == 1)
					{
						sp.imgName = cam1ImgNames[frameId];
						allBackGroundPointsCam1[frameId].push_back(sp);
					}
					else
					{
						sp.imgName = cam2ImgNames[frameId];
						allBackGroundPointsCam2[frameId].push_back(sp);
					}
				}

	//test
	/*
	ofstream fout("o.txt");
	for(auto& pos : allForeGroundScenePoints[0])
		fout << pos.scenePos << endl;
	fout << "====================\n";
	for(auto& pos: allBackGroundPointsCam1[0])
		fout << pos.scenePos << endl;
	fout.close();
	*/
}

CameraModel& Scene3DRecover::getCamModelByFrameId(int camId, int frameId)
{
	int idInSFM;
	if(camId == 1)
		idInSFM = sfmLoader.getCameraByImgName(cam1ImgNames[frameId]);
	else
		idInSFM = sfmLoader.getCameraByImgName(cam2ImgNames[frameId]);
	return sfmLoader.allCameras[idInSFM];
}

void Scene3DRecover::getFilePaths(string& maskFolder, string& mainFolder)
{
	workspaceFolder = mainFolder + string("");
	if (!boost::filesystem::exists(workspaceFolder))
		boost::filesystem::create_directories(workspaceFolder);

	boost::filesystem::recursive_directory_iterator end_iter;
	for (boost::filesystem::recursive_directory_iterator iter(mainFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string();
			string currentImageS = iter->path().filename().string();
			if (iter->path().extension() == string(".jpg") ||
				iter->path().extension() == string(".png")){
				if (currentImageS.find("cam1") != string::npos)
				{
					cam1ImgNames.push_back(currentImagePath);
					cout << currentImagePath << endl;
				}
				else if (currentImageS.find("cam2") != string::npos)
				{
					cam2ImgNames.push_back(currentImagePath);
				}
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;

			}
		}

	}

//	SIFTHandle::updateSIFTfolder(mainFolder, maskFolder, workspaceFolder);
	//Get all the foreground SIFT points
	for (boost::filesystem::recursive_directory_iterator iter(workspaceFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string();
			string currentImageS = iter->path().filename().string();

			if (iter->path().extension() == string(".sift")){
				if (currentImageS.find("cam1") != string::npos)
				{
					cam1SIFTNames.push_back(currentImagePath);
					SIFTFileLoader sfl(currentImagePath);
					allSiftsCam1.push_back(sfl);
				}
				else if (currentImageS.find("cam2") != string::npos)
				{
					cam2SIFTNames.push_back(currentImagePath);
					SIFTFileLoader sfl(currentImagePath);
					allSiftsCam2.push_back(sfl);
				}
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;
			}
		}

	}

	
	for (boost::filesystem::recursive_directory_iterator iter(maskFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string();
			string currentImageS = iter->path().filename().string();
			if (iter->path().extension() == string(".jpg") ||
				iter->path().extension() == string(".png")){
				if (currentImageS.find("cam1") != string::npos)
				{
					cam1MaskNames.push_back(currentImagePath);
				}
				else if (currentImageS.find("cam2") != string::npos)
				{
					cam2MaskNames.push_back(currentImagePath);
				}
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;

			}
			
		}

	}
	

	Mat img = imread(cam1ImgNames[0]);
	FrameH = img.rows;
	FrameW = img.cols;
	cam1Num = cam1ImgNames.size();
	cam2Num = cam2ImgNames.size();
}

void Scene3DRecover::recoverFore3D1F(vector<scenePointOnPair>& out_ScnPoints, string& img1Name, string& img2Name, CameraModel& cam1, CameraModel& cam2, SIFTFileLoader& sfl1, SIFTFileLoader& sfl2)
{
	cv::Mat img1 = imread(img1Name, 1);
	cv::Mat img2 = imread(img2Name, 1);

	/*
	for(auto& pos: sfl1.allFeatsAbs)
		circle(img1, pos.pos, 2, Scalar(255, 0, 0), -1);
	for(auto& pos: sfl2.allFeatsAbs)
		circle(img2, pos.pos, 2, Scalar(255, 0, 0), -1);
	imshow(img1Name, img1);
	imshow(img2Name, img2);
	waitKey(0);
	*/

	vector<DMatch> matchedFeatures;
	vector<KeyPoint> kp1, kp2;
	for(auto& pos : sfl1.allFeatsAbs)
	{
		KeyPoint kp;
		kp.pt = pos.pos;
		kp1.push_back(kp);
	}
	for(auto& pos : sfl2.allFeatsAbs)
	{
		KeyPoint kp;
		kp.pt = pos.pos;
		kp2.push_back(kp);
	}
	getMatchesSIFTLoader(sfl1, sfl2, matchedFeatures);
	Mat out;
	drawMatches(img1, kp1, img2, kp2, matchedFeatures, out);
	imwrite("match.jpg", out);
	resize(out, out, cvSize(out.cols/2, out.rows/2));
	imshow("out", out);
	waitKey(0);

	//cluster using meanshift
	/*
	int rows = matchedFeatures.size();
	int cols = 20; // RGB 6, Pos 4, shift 10 
	float *data = new float[cols*rows];
	for (int i = 0; i < matchedFeatures.size(); i++)
	{
		int id1 = matchedFeatures[i].queryIdx;
		int id2 = matchedFeatures[i].trainIdx;
		Point2f pos1 = sfl1.allFeatsAbs[id1].pos;
		Point2f pos2 = sfl2.allFeatsAbs[id2].pos;
		
		//RGB
		for(int j = 0; j < 3; j++)
		{
			data[i*cols + j] = img1.at<Vec3b>(pos1.y, pos1.x)[j];
			data[i*cols + 3 + j] = img2.at<Vec3b>(pos2.y, pos2.x)[j];
		}
		/*
		//Pos
		data[i*cols + 6] = pos1.x;
		data[i*cols + 7] = pos1.y;
		data[i*cols + 8] = pos2.x;
		data[i*cols + 9] = pos2.y;
		
		//Shift
		int shift_x = pos2.x - pos1.x;
		int shift_y = pos2.y - pos1.y;
		for(int j = 0; j < 7; j++)
		{
			data[i*cols + 10 + j*2] = shift_x;
			data[i*cols + 10 + j*2+1] = shift_y;
		}
	}
	CMeanShift<float> ms;
	ms.setBandwidth(300);
	CEuclideanGeometry<float> geom(cols);
	CVectorPointSet<float> dataPoints(cols, rows, data);
	CVectorPointSet<float> unprunedModes(cols, rows);
	ms.doMeanShift(geom, dataPoints, unprunedModes);
	
	CVectorPointSet<float> prunedModes(cols, 400);
	ms.pruneModes(geom, unprunedModes, prunedModes, 3, 1);
	double kernelDensities[400];
	ms.getKernelDensities(geom, dataPoints, prunedModes, kernelDensities);

	//vector<int> clusterIDEachPoint(dataPoints.size());
	vector<vector<DMatch>> clusteredMatch(50);
	int groupNum = -1;
	for (int i = 0; i < dataPoints.size(); i++)
	{
		int groupID = ms.GroupIDForEverySortedPoint[i];
		int OriInd = ms.index[i];
		//clusters[groupID].push_back(OriInd);
		//clusterIDEachPoint[OriInd] = groupID;
		clusteredMatch[groupID].push_back(matchedFeatures[OriInd]);
		if(groupID > groupNum)
			groupNum = groupID + 1;
	}

	for(int i = 0; i < groupNum; i++)
	{
		Mat out;
		drawMatches(img1, kp1, img2, kp2, clusteredMatch[i], out);
		imshow("out", out);
		waitKey(0);
	}
	*/
	
	int id1, id2; cv::Point2f p1, p2;
//	vector<scenePointOnPair> pSeq;
	cv::Point2f mp(img1.cols / 2, img1.rows / 2);
	for (int k = 0; k < matchedFeatures.size(); k++)
	{
		scenePointOnPair scnPoint;
		id2 = matchedFeatures[k].trainIdx;
		id1 = matchedFeatures[k].queryIdx;
		p1.x = sfl1.allFeatsAbs[id1].pos.x  - mp.x;
		p2.x = sfl2.allFeatsAbs[id2].pos.x  - mp.x;
		p1.y = -(sfl1.allFeatsAbs[id1].pos.y  - mp.y);
		p2.y = -(sfl2.allFeatsAbs[id2].pos.y  - mp.y);

		cv::Point3f scnC = CameraHelper::triangulation(cam1, cam2, p1, p2);
		scnPoint.scenePos = scnC;
		scnPoint.pos2D_1 = sfl1.allFeatsAbs[id1].pos;
		scnPoint.pos2D_2 = sfl2.allFeatsAbs[id2].pos;
		scnPoint.img1Name = img1Name;
		scnPoint.img2Name = img2Name;
		out_ScnPoints.push_back(scnPoint);
		cv::Scalar clr(rand() % 255, rand() % 155 + 100, rand() % 255);
		cv::circle(img1, sfl1.allFeatsAbs[id1].pos, 2, clr, 2, 8, 0);
		cv::circle(img2, sfl2.allFeatsAbs[id2].pos, 2, clr, 2, 8, 0);
		string t = boost::lexical_cast<string>(scnC.x) + string(", ") +
			boost::lexical_cast<string>(scnC.y) + string(", ") +
			boost::lexical_cast<string>(scnC.z);
			
		string text = "Funny text inside the box";
		int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontScale = 0.4;
		int thickness = 1;
		int baseline = 0;
		Size textSize = getTextSize(text, fontFace,
			fontScale, thickness, &baseline);

		// then put the text itself
		//putText(img1, t, sfl1.allFeatsAbs[id1].pos, fontFace, fontScale,
		//	Scalar::all(255), thickness, 8);
		//cout << img1Name << endl << p1.x << ", " << p1.y << endl
		//	<< img2Name << endl << p2.x << ", " << p2.y << endl;

	}
	//cv::imshow("1", img1);
	//cv::imshow("2", img2);
	cv::waitKey(-1);
}

void Scene3DRecover::getMatchesSIFTLoader(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch)
{
	vector<DMatch> matches;
	Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");//创建特征匹配器  
	Mat descriptors1, descriptors2;
	descriptors1.create(cv::Size(128, sfl1.siftNum), CV_32F);
	descriptors2.create(cv::Size(128, sfl2.siftNum), CV_32F);
	
	for (int i = 0; i < sfl1.allFeatVecData.size(); i++)
	{
		for (int j = 0; j < 128; j++)
		{
	//		if (sfl1.ifInMask[i])
				descriptors1.ptr<float>(i)[j] = sfl1.allFeatVecData[i][j];
		}
	}
	for (int i = 0; i < sfl2.allFeatVecData.size(); i++)
	{
		for (int j = 0; j < 128; j++)
		{
		//	if (sfl2.ifInMask[i])
				descriptors2.ptr<float>(i)[j] = sfl2.allFeatVecData[i][j];
		}
	}
	descriptor_matcher->match(descriptors1, descriptors2, matches);
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
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < 0.51 * max_dist)
		{
			outMatch.push_back(matches[i]);
		}
	}
}

void Scene3DRecover::getMatchesSIFTLoader2(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch)
{
	vector<DMatch> matches;
	Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");//创建特征匹配器  
	Mat descriptors1, descriptors2;
	descriptors1.create(cv::Size(128, sfl1.siftNum), CV_32F);
	descriptors2.create(cv::Size(128, sfl2.siftNum), CV_32F);
	
	for (int i = 0; i < sfl1.allFeatVecData.size(); i++)
	{
		for (int j = 0; j < 128; j++)
		{
	//		if (sfl1.ifInMask[i])
				descriptors1.ptr<float>(i)[j] = sfl1.allFeatVecData[i][j];
		}
	}
	for (int i = 0; i < sfl2.allFeatVecData.size(); i++)
	{
		for (int j = 0; j < 128; j++)
		{
		//	if (sfl2.ifInMask[i])
				descriptors2.ptr<float>(i)[j] = sfl2.allFeatVecData[i][j];
		}
	}
	descriptor_matcher->match(descriptors1, descriptors2, matches);
	
	//RANSEC calculate H
	vector<KeyPoint> kp1, kp2;
	for(auto& pos : sfl1.allFeatsAbs)
	{
		KeyPoint kp;
		kp.pt = pos.pos;
		kp1.push_back(kp);
	}
	for(auto& pos : sfl2.allFeatsAbs)
	{
		KeyPoint kp;
		kp.pt = pos.pos;
		kp2.push_back(kp);
	}
	const int MAX_ITERATION = 10000;
	const int SAMPLE_NUM = 4;
	const int MIN_INLIER_NUM = 50;
	const double MAX_ERROR = 3;

	vector<DMatch> best_consensus_set;
	double best_error = 1e10;
	Mat best_H;
	int Num = matches.size();
	cout<<Num<<endl;
	srand(time(0));
	for(int i = 0; i < MAX_ITERATION; i++)
	{
		//train model use random samples
		vector<DMatch> consensus_set;
		vector<Point2f> sel_kp1, sel_kp2;
		for(int j = 0; j < SAMPLE_NUM; j++)
		{
			int index = rand()%Num;
			//cout<<index<<endl;
			sel_kp1.push_back(kp1[matches[index].queryIdx].pt);
			sel_kp2.push_back(kp2[matches[index].trainIdx].pt);
		}
		Mat H = findHomography(sel_kp1, sel_kp2);
		//search inlier
		for(unsigned j = 0; j < Num; j++)
		{
			Point2f p2_t = transform(H, kp1[matches[j].queryIdx].pt);
			Point2f p2 = kp2[matches[j].trainIdx].pt;
			double error = (p2.x - p2_t.x) * (p2.x - p2_t.x) + (p2.y - p2_t.y) * (p2.y - p2_t.y);
			error = sqrt(error);
			if(error < MAX_ERROR)
				consensus_set.push_back(matches[j]);
		}
		//check model
		//cout<<"size "<<consensus_set.size()<<endl;
		if(consensus_set.size() < MIN_INLIER_NUM)
			continue;
		sel_kp1.clear();
		sel_kp2.clear();
		for(unsigned j = 0; j < consensus_set.size(); j++)
		{
			sel_kp1.push_back(kp1[consensus_set[j].queryIdx].pt);
			sel_kp2.push_back(kp2[consensus_set[j].trainIdx].pt);
		}
		Mat better_H = findHomography(sel_kp1, sel_kp2);
		double better_error = 0;
		for(unsigned j = 0; j < consensus_set.size(); j++)
		{
			Point2f p2_t = transform(better_H, kp1[consensus_set[j].queryIdx].pt);
			Point2f p2 = kp2[consensus_set[j].trainIdx].pt;
			double error = (p2.x - p2_t.x) * (p2.x - p2_t.x) + (p2.y - p2_t.y) * (p2.y - p2_t.y);
			error = sqrt(error);
			better_error += error;
		}
		//cout<<"error "<<better_error<<endl;
		if(better_error < best_error)
		{
			best_error = better_error;
			best_H = better_H;
			best_consensus_set = consensus_set;
		}
	}


	//筛选出较好的匹配点  
	for (int i = 0; i < best_consensus_set.size(); i++)
	{
		outMatch.push_back(best_consensus_set[i]);
	}
}

Point2f Scene3DRecover::transform(Mat &H, Point2f srcp)
{
	double src[3] = {srcp.x, srcp.y, 1};
	double dest[3] = {0, 0, 0};
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			dest[i] += H.at<double>(i,j) * src[j];
	return Point2f(dest[0]/dest[2], dest[1]/dest[2]);
}


Scene3DRecover::Scene3DRecover()
{
}


Scene3DRecover::~Scene3DRecover()
{
}

void Scene3DRecover::mapCam2Frame()
{
	for(unsigned i = 0; i < cam1ImgNames.size(); i++)
	{
		int camId = sfmLoader.getCameraByImgName(cam1ImgNames[i]);
		frame2Cam.insert(make_pair(make_pair(1, i), camId));
	}
	for(unsigned i = 0; i < cam2ImgNames.size(); i++)
	{
		int camId = sfmLoader.getCameraByImgName(cam2ImgNames[i]);
		frame2Cam.insert(make_pair(make_pair(2, i), camId));
	}

	for(auto& pair : frame2Cam)
		cam2Frame.insert(make_pair(pair.second, pair.first));
}