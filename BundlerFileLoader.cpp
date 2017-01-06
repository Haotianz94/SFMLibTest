#include "BundlerFileLoader.h"
#include <boost/filesystem.hpp>


BundlerFileLoader::BundlerFileLoader()
{
}


BundlerFileLoader::BundlerFileLoader(string& cameraFileName, string& bundlerFileName)
{
	if (!boost::filesystem::exists(cameraFileName) || !boost::filesystem::exists(bundlerFileName))
		return;

	ifstream ifCameraList(cameraFileName, std::ios::in);
	ifstream ifBundler(bundlerFileName, std::ios::in);
	
	const int LINE_LENGTH = 100;
	char str[LINE_LENGTH];
	int count = 0;
	while (1)
	{
		if (count < 16)
			count++;
		else
			break;
		ifCameraList.getline(str, LINE_LENGTH);
	}
	int cameraNum;
	ifCameraList >> cameraNum;
	for (int i = 0; i < cameraNum; i++)
	{
		string s;
		ifCameraList >> s;
		ifCameraList >> s;
		viewImgFileName.push_back(s);
		cv::Mat img;
		img = cv::imread(s, 1);
		allImgs.push_back(img);
		for (int k = 0; k < 12; k++)
		{
			ifCameraList.getline(str, LINE_LENGTH);
		}
	}

	while (!ifCameraList.eof())
	{
		string s;
		ifCameraList >> s;
		if (s.length() < 2)
			break;
		viewImgFileName.push_back(s);
		cv::Mat img;
		img = cv::imread(s, 1);
		allImgs.push_back(img);
	}
	while (!ifBundler.eof())
	{
		string s;
		ifBundler >> s;
		ifBundler >> s;
		ifBundler >> s;
		ifBundler >> s;
		if (s.length() < 1)
			break;
		ifBundler >> this->viewNum;
		if (viewNum != cameraNum)
			cout << "WARNING!!!!!!!!!!!!!!!! Not compatible camera number " << endl;
		ifBundler >> this->featNum;

		REPORT(cameraNum);
		REPORT(viewNum);
		
		//for all the cameras
		for (int i = 0; i < viewNum; i++)
		{
			CameraModel cm;
			ifBundler >> cm.focallength;

			REPORT(cm.focallength);
			ifBundler >> cm.distortX;
			ifBundler >> cm.distortY;
			cm.rotation.create(cv::Size(3, 3), CV_64F);
			cm.translation.create(cv::Size(1, 3), CV_64F);
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					ifBundler >> cm.rotation.ptr<double>(r)[c];
				}
			}
			for (int r = 0; r < 3; r++)
			{
				ifBundler >> cm.translation.ptr<double>(r)[0];
			}
			this->allCameras.push_back(cm);
		}

		//for all the points
		for (int i = 0; i < featNum; i++)
		{
			OneFeatureInWholeScene oneFeat;
			ifBundler >> oneFeat.position3D.x;
			ifBundler >> oneFeat.position3D.y;
			ifBundler >> oneFeat.position3D.z;

			ifBundler >> oneFeat.sceneColor[0];
			ifBundler >> oneFeat.sceneColor[1];
			ifBundler >> oneFeat.sceneColor[2];

			ifBundler >> oneFeat.numOfVisibelCam;

			for (int k = 0; k < oneFeat.numOfVisibelCam; k++)
			{
				FeatureOneView featOneView;
				ifBundler >> featOneView.cameraIndex;
				ifBundler >> featOneView.siftIndex;
				ifBundler >> featOneView.position2D.x;
				ifBundler >> featOneView.position2D.y;
				oneFeat.featInAllViews.push_back(featOneView);
			}
			this->allFeats.push_back(oneFeat);
		}
	}
}

void BundlerFileLoader::init(string& cameraFileName, string& bundlerFileName)
{
	ifstream ifCameraList(cameraFileName, std::ios::in);
	ifstream ifBundler(bundlerFileName, std::ios::in);


	const int LINE_LENGTH = 100;
	char str[LINE_LENGTH];
	int count = 0;
	while (1)
	{
		if (count < 16)
			count++;
		else
			break;
		ifCameraList.getline(str, LINE_LENGTH);
	}
	int cameraNum;
	ifCameraList >> cameraNum;
	for (int i = 0; i < cameraNum; i++)
	{
		string s;
		ifCameraList >> s;
		ifCameraList >> s;
		viewImgFileName.push_back(s);
		cv::Mat img;
		img = cv::imread(s, 1);
		allImgs.push_back(img);
		for (int k = 0; k < 12; k++)
		{
			ifCameraList.getline(str, LINE_LENGTH);
		}
	}

	while (!ifCameraList.eof())
	{
		string s;
		ifCameraList >> s;
		if (s.length() < 2)
			break;
		viewImgFileName.push_back(s);
		cv::Mat img;
		img = cv::imread(s, 1);
		allImgs.push_back(img);
	}
	while (!ifBundler.eof())
	{
		string s;
		ifBundler >> s;
		ifBundler >> s;
		ifBundler >> s;
		ifBundler >> s;
		if (s.length() < 1)
			break;
		ifBundler >> this->viewNum;
		if (viewNum != cameraNum)
			cout << "WARNING!!!!!!!!!!!!!!!! Not compatible camera number " << endl;
		ifBundler >> this->featNum;

		//for all the cameras
		for (int i = 0; i < viewNum; i++)
		{
			CameraModel cm;
			ifBundler >> cm.focallength;
			ifBundler >> cm.distortX;
			ifBundler >> cm.distortY;
			cm.rotation.create(cv::Size(3, 3), CV_64F);
			cm.translation.create(cv::Size(1, 3), CV_64F);
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					ifBundler >> cm.rotation.ptr<double>(r)[c];
				}
			}
			for (int r = 0; r < 3; r++)
			{
				ifBundler >> cm.translation.ptr<double>(r)[0];
			}
			this->allCameras.push_back(cm);
		}

		//for all the points
		for (int i = 0; i < featNum; i++)
		{
			OneFeatureInWholeScene oneFeat;
			ifBundler >> oneFeat.position3D.x;
			ifBundler >> oneFeat.position3D.y;
			ifBundler >> oneFeat.position3D.z;

			ifBundler >> oneFeat.sceneColor[0];
			ifBundler >> oneFeat.sceneColor[1];
			ifBundler >> oneFeat.sceneColor[2];

			ifBundler >> oneFeat.numOfVisibelCam;

			for (int k = 0; k < oneFeat.numOfVisibelCam; k++)
			{
				FeatureOneView featOneView;
				ifBundler >> featOneView.cameraIndex;
				ifBundler >> featOneView.siftIndex;
				ifBundler >> featOneView.position2D.x;
				ifBundler >> featOneView.position2D.y;
				oneFeat.featInAllViews.push_back(featOneView);
			}
			this->allFeats.push_back(oneFeat);
		}
	}
}

int BundlerFileLoader::getCameraByImgName(string& imgName)
{
	int x = -1;
	for (int i = 0; i < viewImgFileName.size(); i++)
	{
		string substr1 = viewImgFileName[i].substr(viewImgFileName[i].length() - 10, viewImgFileName[i].length() - 1);
		string substr2 = imgName.substr(imgName.length()-10, imgName.length()-1);
		//cout << substr1 << ' ' << substr2 << endl;
		if (substr1 == substr2)
		{
			x = i;
			break;
		}
	}
	return x;
}

BundlerFileLoader::~BundlerFileLoader()
{
}
