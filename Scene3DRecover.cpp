#include "Scene3DRecover.h"
#include "Logger.h"
#include "Configer.h"
/*
#include "Geometry/Include/EuclideanGeometry.h"
#include "Geometry/Include/SE3Geometry.h"
#include "Geometry/Include/GrassmannGeometry.h"
#include "Geometry/Include/EssentialGeometry.h"
#include "Geometry/Include/DTIAffineInvariantGeometry.h"
#include "MeanShiftLib/Include/VectorPointSet.h"
#include "MeanShiftLib/Include/EssentialPointSet.h"
#include "MeanShiftLib/Include/MeanShift.h"
*/

using namespace std;
void Scene3DRecover::getForeGround3DAllFrames()
{
	LOG << "Loading Foreground features...\n\n";
	for (int i = 0; i < matchedFramesID.size(); i++)
	{
		int id1 = matchedFramesID[i].first;
		int id2 = matchedFramesID[i].second;
		cout << cam1FeatFGNames[id1] << endl;
		SIFTFileLoader sfl1(cam1FeatFGNames[id1]);
		allSiftsCam1.push_back(sfl1);

		cout << cam1FeatFGNames[id2] << endl;
		SIFTFileLoader sfl2(cam2FeatFGNames[id2]);
		allSiftsCam2.push_back(sfl2);
	}

	Configer::getConfiger()->getInt("param", "trackNum", trackNum);
	if(trackNum != 0)
	{
		LOG << "Tracking Fg features in time sequence...\n\n";
		trackForeGround(allSiftsCam1);
		trackForeGround(allSiftsCam2);
	}

	LOG << "Recovering Fg 3D using triangulation...\n\n";
	for (int i = 0; i < matchedFramesID.size(); i++)
	{
		vector<scenePointOnPair> ScnPoints;
		//if there is no foreground in one frame, continue
		if( std::find(lackFramesPair.begin(), lackFramesPair.end(), i) != lackFramesPair.end() )
		{
			allForeGroundScenePoints.push_back(ScnPoints);
			continue;
		}
		cout << matchedFramesID.size() << endl;
		int id1 = matchedFramesID[i].first;
		int idInSFM1 = frame2Cam[make_pair(1, id1)];
		//In the bundler file, the imgs are re-ordered according to the camera parameters
		int id2 = matchedFramesID[i].second;
		int idInSFM2 = frame2Cam[make_pair(2, id2)];
		cout << "Frame ID pair:" << id1 << "," << id2 << endl;
		cout << "In sfm file id: " << idInSFM1 << ", " << idInSFM2 << endl;
		cout << sfmLoader.allCameras.size() << endl;
		this->recoverFore3D1F(ScnPoints, sfmLoader.allCameras[idInSFM1], sfmLoader.allCameras[idInSFM2], id1, id2);
		allForeGroundScenePoints.push_back(ScnPoints);

		//if there are not enough matched pairs in foreground 
		if(ScnPoints.size() < 10 && i != 0 && i != matchedFramesID.size()-1)
			lackFramesPair.push_back(i);
		
		LOG << "Recover Foreground 3D in "<< i << " th frame\n";
		//break;//*************
	}
}

void Scene3DRecover::recoverFore3D1F(vector<scenePointOnPair>& out_ScnPoints, CameraModel& cam1, CameraModel& cam2, int frameId1, int frameId2)
{
	string img1Name = cam1ImgNames[frameId1];
	string img2Name = cam2ImgNames[frameId2];
	cv::Mat img1 = imread(img1Name, 1);
	cv::Mat img2 = imread(img2Name, 1);
	SIFTFileLoader sfl1 = allSiftsCam1[frameId1];
	SIFTFileLoader sfl2 = allSiftsCam2[frameId2];

	vector<DMatch> matchedFeatures;
	//get foreground feature matches from feature distance
	getMatchesSIFTLoader(sfl1, sfl2, matchedFeatures);
	LOG << "Matches after view match: " << (int)matchedFeatures.size() << "\n";

	//get foreground feature matches from Ransac
	//getMatchesSIFTLoader_Ransac(sfl1, sfl2, matchedFeatures);

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
	
	//remove points not in tracked time sequence
	if(trackNum != 0)
	{

	int id1, id2, i = 0;
	while(i < matchedFeatures.size())
	{
		id2 = matchedFeatures[i].trainIdx;
		id1 = matchedFeatures[i].queryIdx;

		bool trackFlag = true;
		int trackedId1 = id1, trackedId2 = id2;
		int trackFId1 = frameId1, trackFId2 = frameId2;
		for(int i = 0; i < trackNum; i++)
		{
			if(trackFId1 >= cam1Num-1 || trackFId2 >= cam2Num-1)
				break;
			trackedId1 = allSiftsCam1[trackFId1++].trackNextFrame[trackedId1];
			trackedId2 = allSiftsCam2[trackFId2++].trackNextFrame[trackedId2];
			if(trackedId1 == -1 || trackedId2 == -1)
			{
				trackFlag = false;
				break;
			}
		}
		trackedId1 = id1, trackedId2 = id2;
		trackFId1 = frameId1, trackFId2 = frameId2;
		for(int i = 0; i < trackNum; i++)
		{
			if(trackFId1 <= 0 || trackFId2 <= 0)
				break;
			trackedId1 = allSiftsCam1[trackFId1--].trackLastFrame[trackedId1];
			trackedId2 = allSiftsCam2[trackFId2--].trackLastFrame[trackedId2];
			if(trackedId1 == -1 || trackedId2 == -1)
			{
				trackFlag = false;
				break;
			}
		}
		if(!trackFlag)
			matchedFeatures.erase(matchedFeatures.begin() + i);
		else
			++i;
	}

	LOG << "Matches after sequence tracking: " << (int)matchedFeatures.size() << "\n";
	}

	//remove outliers using homography
	bool isRemovingOutlier;
	Configer::getConfiger()->getBool("param", "isRemovingOutlier", isRemovingOutlier);
	if(isRemovingOutlier && matchedFeatures.size() > 10)
	{
	
	vector<Point2f> sel_kp1, sel_kp2;
	for(auto& m : matchedFeatures)
	{
		sel_kp1.push_back(sfl1.allFeatsAbs[m.queryIdx].pos);
		sel_kp2.push_back(sfl2.allFeatsAbs[m.trainIdx].pos);
	}
	Mat H = findHomography(sel_kp1, sel_kp2);
	double max_error = 400;
	Configer::getConfiger()->getDouble("param", "max_error", max_error);
	
	int i = 0;
	while(i < matchedFeatures.size())
	{
		int id1 = matchedFeatures[i].queryIdx;
		int id2 = matchedFeatures[i].trainIdx;
		Point2f p2_t = transform(H, sfl1.allFeatsAbs[id1].pos);
		Point2f p2 = sfl2.allFeatsAbs[id2].pos;
		double error = (p2.x - p2_t.x) * (p2.x - p2_t.x) + (p2.y - p2_t.y) * (p2.y - p2_t.y);
		if(error > max_error)
			matchedFeatures.erase(matchedFeatures.begin() + i);
		else
			++i;
	}

	LOG << "Matches after removing outliers: " << (int)matchedFeatures.size() << "\n";
	}

	cv::Point2f p1, p2;
	cv::Point2f mp(img1.cols / 2.0f, img1.rows / 2.0f);
	for(auto& m : matchedFeatures)
	{
		int id1 = m.queryIdx;
		int id2 = m.trainIdx;

		//adjust coordinate system
		p1.x = sfl1.allFeatsAbs[id1].pos.x  - mp.x;
		p2.x = sfl2.allFeatsAbs[id2].pos.x  - mp.x;
		p1.y = -(sfl1.allFeatsAbs[id1].pos.y  - mp.y);
		p2.y = -(sfl2.allFeatsAbs[id2].pos.y  - mp.y);
		//triangulation
		cv::Point3f scnC = CameraHelper::triangulation(cam1, cam2, p1, p2);
		scenePointOnPair scnPoint;
		scnPoint.scenePos = scnC;
		scnPoint.pos2D_1 = sfl1.allFeatsAbs[id1].pos;
		scnPoint.pos2D_2 = sfl2.allFeatsAbs[id2].pos;
		scnPoint.img1Name = img1Name;
		scnPoint.img2Name = img2Name;
		out_ScnPoints.push_back(scnPoint);
	}

	//draw matched after matching in time sequence and two views
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

	Mat out;
	drawMatches(img1, kp1, img2, kp2, matchedFeatures, out);
	string baseFolder, folder, No, Cmvs;
	Configer::getConfiger()->getString("input", "baseFolder", baseFolder);
	Configer::getConfiger()->getString("input", "folder", folder);
	Configer::getConfiger()->getString("input", "No", No);
	string matchFolder = baseFolder + folder + string("\\") + No + string("\\out\\match\\");
	char num[10];
	sprintf_s(num, "%d", frameId1);
	imwrite((matchFolder + string(num) + string(".jpg")), out);
}

void Scene3DRecover::getMatchesSIFTLoader(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch)
{
	vector<DMatch> matches;
	Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");//创建特征匹配器  
	Mat descriptors1, descriptors2;
	
	string featureFG;
	Configer::getConfiger()->getString("input", "featureFG", featureFG);

	if(featureFG == "SURF")
	{
		descriptors1.create(cv::Size(64, sfl1.siftNum), CV_32F);
		descriptors2.create(cv::Size(64, sfl2.siftNum), CV_32F);
		for (int i = 0; i < sfl1.allFeatVecVal.size(); i++)
		{
			for (int j = 0; j < 64; j++)
			{
				descriptors1.at<float>(i, j) = sfl1.allFeatVecVal[i][j];
			}
		}
		for (int i = 0; i < sfl2.allFeatVecVal.size(); i++)
		{
			for (int j = 0; j < 64; j++)
			{
				descriptors2.at<float>(i, j) = sfl2.allFeatVecVal[i][j];
			}
		}
	}
	else if(featureFG == "SIFT")
	{
		descriptors1.create(cv::Size(128, sfl1.siftNum), CV_32F);
		descriptors2.create(cv::Size(128, sfl2.siftNum), CV_32F);
		for (int i = 0; i < sfl1.allFeatVecData.size(); i++)
		{
			for (int j = 0; j < 128; j++)
			{
				descriptors1.ptr<float>(i)[j] = sfl1.allFeatVecData[i][j];
			}
		}
		for (int i = 0; i < sfl2.allFeatVecData.size(); i++)
		{
			for (int j = 0; j < 128; j++)
			{
				descriptors2.ptr<float>(i)[j] = sfl2.allFeatVecData[i][j];
			}
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
	//cout << "最大距离：" << max_dist << endl;
	//cout << "最小距离：" << min_dist << endl;

	//筛选出较好的匹配点 
	double siftThreshold;
	Configer::getConfiger()->getDouble("param", "siftThreshold", siftThreshold);
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < siftThreshold * max_dist)
		{
			outMatch.push_back(matches[i]);
		}
	}
}

void Scene3DRecover::getMatchesSIFTLoader_Ransac(SIFTFileLoader& sfl1, SIFTFileLoader& sfl2, vector<DMatch>& outMatch)
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
		for(int j = 0; j < Num; j++)
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

void Scene3DRecover::getBackGround3DAllFrames()
{
	LOG << "Loading all background 3D from bundler...\n\n";

	int reconFrameNum = 30;
	Configer::getConfiger()->getInt("input", "reconFrameNum", reconFrameNum);
	allBackGroundPointsCam1.resize(reconFrameNum);
	allBackGroundPointsCam2.resize(reconFrameNum);
	
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
	int frameNum = 30;
	Configer::getConfiger()->getInt("input", "frameNum", frameNum);

	LOG << "Loading all image file path...\n\n";
	boost::filesystem::recursive_directory_iterator end_iter;
	for (boost::filesystem::recursive_directory_iterator iter(mainFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string();

#ifdef OLD_BOOST
			string currentImageS = iter->path().filename();
#else
			string currentImageS = iter->path().filename().string();
#endif

			if (iter->path().extension() == string(".jpg") ||
				iter->path().extension() == string(".png")){
				if (currentImageS.find("cam1") != string::npos)
				{
					cam1ImgNames.push_back(currentImagePath);
					//cout << currentImagePath << endl;
				}
				else if (currentImageS.find("cam2") != string::npos)
				{
					cam2ImgNames.push_back(currentImagePath);
					//cout << currentImagePath << endl;
				}
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;

			}
		}

	}

	
	LOG << "Loading all foreground feature file path...\n\n";
	string featureFG;
	Configer::getConfiger()->getString("input", "featureFG", featureFG);
	string extention;
	if(featureFG == "SIFT")
		extention = ".sift";
	else if(featureFG == "SURF")
		extention = ".surf";

	for (boost::filesystem::recursive_directory_iterator iter(mainFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string();
#ifdef OLD_BOOST
			string currentImageS = iter->path().filename();
#else
			string currentImageS = iter->path().filename().string();
#endif
			if (iter->path().extension() == extention){
				if (currentImageS.find("cam1") != string::npos)
				{
					cam1FeatFGNames.push_back(currentImagePath);
				}
				else if (currentImageS.find("cam2") != string::npos)
				{
					cam2FeatFGNames.push_back(currentImagePath);
				}
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;
			}
		}

	}


	LOG << "Loading all mask file path...\n\n";
	for (boost::filesystem::recursive_directory_iterator iter(maskFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string(); 
#ifdef OLD_BOOST
				string currentImageS = iter->path().filename();
#else
			string currentImageS = iter->path().filename().string();
#endif
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
}

void Scene3DRecover::createNewCamPath()
{
	LOG << "Creating new camera path...\n\n";
	for (unsigned i = 0; i < matchedFramesID.size(); i++)
	{
		int id1 = matchedFramesID[i].first;
		int idInSFM1 = frame2Cam[make_pair(1, id1)];
		//In the bundler file, the imgs are re-ordered according to the camera parameters
		int id2 = matchedFramesID[i].second;
		int idInSFM2 = frame2Cam[make_pair(2, id2)];

		newCamPath.push_back(sfmLoader.allCameras[idInSFM1].getMedian(sfmLoader.allCameras[idInSFM2]));
	}

	LOG << "Filtering new camera path...\n\n";
	//Median filter
	int iteration = 3;
	int step = 1;
	Configer::getConfiger()->getInt("newCamPath", "iteration", iteration);
	Configer::getConfiger()->getInt("newCamPath", "step", step);
	for(int k = 0; k < iteration; k++)
	{
		for(int i = 0; i < newCamPath.size(); i++)
		{
			Mat rotation = Mat(3, 3, CV_64F, 0.0);
			Mat translation = Mat(3, 1, CV_64F, 0.0);
			double focal = 0, distortX = 0, distortY = 0;
			for(int j = -step; j <= step; j++)
			{
				if(i+j < 0)
				{
					rotation += newCamPath.front().rotation;
					translation += newCamPath.front().translation;
					focal += newCamPath.front().focallength;
					distortX += newCamPath.front().distortX;
					distortY += newCamPath.front().distortY;
				}
				else if(i+j > (int)newCamPath.size()-1)
				{
					rotation += newCamPath.back().rotation;
					translation += newCamPath.back().translation;
					focal += newCamPath.back().focallength;
					distortX += newCamPath.back().distortX;
					distortY += newCamPath.back().distortY;
				}
				else
				{
					rotation += newCamPath[i+j].rotation;
					translation += newCamPath[i+j].translation;
					focal += newCamPath[i+j].focallength;
					distortX += newCamPath[i+j].distortX;
					distortY += newCamPath[i+j].distortY;
				}
			}
			newCamPath[i].rotation = rotation / (2*step+1);
			newCamPath[i].translation = translation / (2*step+1);
			newCamPath[i].focallength = focal / (2*step+1);
			newCamPath[i].distortX = distortX / (2*step+1);
			newCamPath[i].distortY = distortY / (2*step+1);
		}
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
		cout << cam1ImgNames[i] << endl;
		frame2Cam.insert(make_pair(make_pair(1, i), camId));
//		cout << "cam 1 pair: " << i << ", " << camId << endl;
	}
	for(unsigned i = 0; i < cam2ImgNames.size(); i++)
	{
		int camId = sfmLoader.getCameraByImgName(cam2ImgNames[i]);
		cout << cam2ImgNames[i] << endl;
		frame2Cam.insert(make_pair(make_pair(2, i), camId));
//		cout << "cam 2 pair: " << i << ", " << camId << endl;
	}

	for(auto& pair : frame2Cam)
		cam2Frame.insert(make_pair(pair.second, pair.first));
}

void Scene3DRecover::trackForeGround(vector<SIFTFileLoader>& allSiftsCam)
{
	for(int i = 0; i < allSiftsCam.size()-1; i++)
	{
		vector<DMatch> matchedFeatures;
		//get foreground feature matches from feature distance
		getMatchesSIFTLoader(allSiftsCam[i], allSiftsCam[i+1], matchedFeatures);
		for(auto& match : matchedFeatures)
		{
			allSiftsCam[i].trackNextFrame[match.queryIdx] = match.trainIdx;
			allSiftsCam[i+1].trackLastFrame[match.trainIdx] = match.queryIdx;
		}
	}
}