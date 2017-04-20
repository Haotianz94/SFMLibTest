#include "SIFTFileLoader.h"
#include "Logger.h"

/*
[Header][Location Data][Descriptor Data][EOF]

[Header] = int[5] = { name, version, npoint, 5, 128 };
name = ('S' + ('I' << 8) + ('F' << 16) + ('T' << 24));
version = ('V' + ('4' << 8) + ('.' << 16) + ('0' << 24)); or('V' + ('5' << 8) + ('.' << 16) + ('0' << 24)) if containing color info
npoint = number of features.

[Location Data]  is a npoint x 5 float matrix and each row  is[x, y, color, scale, orientation].
Write color by casting the float to unsigned char[4]
scale & orientation are only used for visualization, so you can simply write 0 for them

* Sort features in the order of decreasing importance, since VisualSFM may use only part of those features.
* VisualSFM sorts the features in the order of decreasing scales.

[Descriptor Data] is a npoint x 128 unsigned char matrix.Note the feature descriptors are normalized to 512.

[EOF]  int eof_marker = (0xff + ('E' << 8) + ('O' << 16) + ('F' << 24));
*/


SIFTFileLoader::SIFTFileLoader()
{
}


SIFTFileLoader::SIFTFileLoader(string& siftFileName)
{
	FILE *f;
	char a, b, c, d, e;
	fstream streamF;
	streamF.open(siftFileName, ios_base::in | ios_base::binary);

	int temp1, temp2, readInt;
	streamF.read((char *)(&temp1), sizeof(temp1));
	streamF.read((char *)(&temp2), sizeof(temp2));

	streamF.read((char *)(&siftNum), sizeof(siftNum));
	streamF.read((char *)(&temp1), sizeof(temp1));
	streamF.read((char *)(&temp2), sizeof(temp2));

	for (int i = 0; i < siftNum; i++)
	{
		siftAbstract oneAbs;
		streamF.read((char *)(&oneAbs.pos.x), sizeof(oneAbs.pos.x));
		streamF.read((char *)(&oneAbs.pos.y), sizeof(oneAbs.pos.y));
		streamF.read((char *)(&oneAbs.colorStore4b), sizeof(oneAbs.colorStore4b));
		streamF.read((char *)(&oneAbs.scale), sizeof(oneAbs.scale));
		streamF.read((char *)(&oneAbs.orientation), sizeof(oneAbs.orientation));
    	allFeatsAbs.push_back(oneAbs);
		trackLastFrame.push_back(-1);
		trackNextFrame.push_back(-1);
	}
	
	for (int i = 0; i < siftNum; i++)
	{
		vector<unsigned char> featData;
		for (int k = 0; k < 128; k++)
		{
			unsigned char fff;
			streamF.read((char *)(&fff), sizeof(fff));
			featData.push_back(fff);
		}
		allFeatVecData.push_back(featData);
	}
	streamF.read(&a, sizeof(a));
	streamF.read(&b, sizeof(a));
	streamF.read(&c, sizeof(a));
	streamF.read(&a, sizeof(a));

	cout << "-------Finish reading a sift file" << endl;
	streamF.close();
}

void SIFTFileLoader::removeSIFTOutofMask(cv::Mat& mask, string& newfileName)
{
	int count = 0;
	ifInMask.resize(siftNum);
	for (int i = 0; i < siftNum; i++)
	{
		int x = allFeatsAbs[i].pos.x;
		int y = allFeatsAbs[i].pos.y;
		if(x < 0 || x > mask.cols-1 || y < 0 || y > mask.rows-1)
		{
			ifInMask[i] = false;
			continue;
		}

		ifInMask[i] = false;
		if (mask.at<uchar>(y, x) > 127)
		{
			ifInMask[i] = true;
			count++;
		}
	}

	ofstream outfile(newfileName.c_str(), ios::binary);
	char tempChar = 'S';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'I';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'F';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'T';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'V';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = '4';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = '.';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = '0';	outfile.write((char*)&tempChar, sizeof(tempChar));
	outfile.write((char*)&count, sizeof(count));

	int tempInt;
	tempInt = 5;	outfile.write((char*)&tempInt, sizeof(tempInt));
	tempInt = 128;	outfile.write((char*)&tempInt, sizeof(tempInt));

	for (int i = 0; i < siftNum; i++)
	{
		if (!(ifInMask[i]))
			continue;
		refinedOriIDs.push_back(i);
		siftAbstract& oneAbs = allFeatsAbs[i];
		outfile.write((char *)(&oneAbs.pos.x), sizeof(oneAbs.pos.x));
		outfile.write((char *)(&oneAbs.pos.y), sizeof(oneAbs.pos.y));
		outfile.write((char *)(&oneAbs.colorStore4b), sizeof(oneAbs.colorStore4b));
		outfile.write((char *)(&oneAbs.scale), sizeof(oneAbs.scale));
		outfile.write((char *)(&oneAbs.orientation), sizeof(oneAbs.orientation));
	}

	for (int i = 0; i < siftNum; i++)
	{
		if (!(ifInMask[i]))
			continue;
		vector<unsigned char>& featData = allFeatVecData[i];
		for (int k = 0; k < 128; k++)
		{
			outfile.write((char *)(&(featData[k])), sizeof(featData[k]));
		}
	}

	tempChar = ' ';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'E';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'O';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'F';	outfile.write((char*)&tempChar, sizeof(tempChar));
	keepNum = count;
}



void SIFTFileLoader::removeSIFTinMask(cv::Mat& mask, string& newfileName)
{
	int count = 0;
	ifInMask.resize(siftNum);
	for (int i = 0; i < siftNum; i++)
	{
		int x = allFeatsAbs[i].pos.x;
		int y = allFeatsAbs[i].pos.y;
		if(x < 0 || x > mask.cols-1 || y < 0 || y > mask.rows-1)
		{
			ifInMask[i] = true;
			continue;
		}
		
		ifInMask[i] = false;
		if (mask.at<uchar>(y, x) > 127)
			ifInMask[i] = true;
		else
			count++;
	}

	ofstream outfile(newfileName.c_str(), ios::binary);
	char tempChar = 'S';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'I';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'F';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'T';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'V';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = '4';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = '.';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = '0';	outfile.write((char*)&tempChar, sizeof(tempChar));
	outfile.write((char*)&count, sizeof(count));

	int tempInt;
	tempInt = 5;	outfile.write((char*)&tempInt, sizeof(tempInt));
	tempInt = 128;	outfile.write((char*)&tempInt, sizeof(tempInt));

	for (int i = 0; i < siftNum; i++)
	{
		if (ifInMask[i])
			continue;
		refinedOriIDs.push_back(i);
		siftAbstract& oneAbs = allFeatsAbs[i];
		outfile.write((char *)(&oneAbs.pos.x), sizeof(oneAbs.pos.x));
		outfile.write((char *)(&oneAbs.pos.y), sizeof(oneAbs.pos.y));
		outfile.write((char *)(&oneAbs.colorStore4b), sizeof(oneAbs.colorStore4b));
		outfile.write((char *)(&oneAbs.scale), sizeof(oneAbs.scale));
		outfile.write((char *)(&oneAbs.orientation), sizeof(oneAbs.orientation));
		refinedFeatsAbs.push_back(oneAbs);
	}

	for (int i = 0; i < siftNum; i++)
	{
		if (ifInMask[i])
			continue;
		vector<unsigned char>& featData = allFeatVecData[i];
		for (int k = 0; k < 128; k++)
		{
			outfile.write((char *)(&(featData[k])), sizeof(featData[k]));
		}
		refinedFeatVecData.push_back(featData);
	}

	tempChar = ' ';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'E';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'O';	outfile.write((char*)&tempChar, sizeof(tempChar));
	tempChar = 'F';	outfile.write((char*)&tempChar, sizeof(tempChar));
	keepNum = count;
}

SIFTFileLoader::~SIFTFileLoader()
{
}
/*

using namespace std;
struct student
{
	char name[20];
	int num;
	int age;
	char sex;
};
int main()
{
	student stud[3] = { "Li", 1001, 18, 'f', "Fun", 1002, 19, 'm', "Wang", 1004, 17, 'f' };
	ofstream outfile("stud.dat", ios::binary);
	if (!outfile)
	{
		cerr << "open error!" << endl;
		abort();//ÍË³ö³ÌÐò
	}
	for (int i = 0; i < 3; i++)
		outfile.write((char*)&stud[i], sizeof(stud[i]));
	outfile.close();
	return 0;
	}*/

void SIFTHandle::updateSIFTfolder(string& imgFolder, string& maskFolder, string& fgFolder, string& bgFolder)
{
		if (!boost::filesystem::exists(fgFolder))
			boost::filesystem::create_directories(fgFolder);
		if (!boost::filesystem::exists(bgFolder))
			boost::filesystem::create_directories(bgFolder);
		vector<string> imgNameList, siftNameList, maskNameList, basenameList,
			bgSiftList, bgNameList, fgSiftList, fgNameList;
		

		//missing code for iterate the two folders.
		//
		boost::filesystem::recursive_directory_iterator end_iter;
		for (boost::filesystem::recursive_directory_iterator iter(imgFolder); iter != end_iter; iter++)
		{
			if (!boost::filesystem::is_directory(*iter)){
				string currentImagePath = iter->path().string();
				string currentImageS = iter->path().filename().string();
				if (iter->path().extension() == string(".jpg") ||
					iter->path().extension() == string(".png")){
					imgNameList.push_back(currentImagePath);
					basenameList.push_back(iter->path().filename().string());
					//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;
					
				}
				if (iter->path().extension() == string(".sift")){
					siftNameList.push_back(currentImagePath);
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
					maskNameList.push_back(currentImagePath);
					//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;

				}
			}

		}
		int fileNum = imgNameList.size();
		bgNameList.resize(fileNum);
		bgSiftList.resize(fileNum);
		fgNameList.resize(fileNum);
		fgSiftList.resize(fileNum);
		for (int i = 0; i < fileNum; i++)
		{
			cv::Mat img = cv::imread(imgNameList[i], 1);
			cv::Mat mask = cv::imread(maskNameList[i], 0);

			SIFTFileLoader sfl(siftNameList[i]);
			bgSiftList[i] = bgFolder + string("\\") + basenameList[i].substr(0, basenameList[i].length()-4) + string(".sift");
			bgNameList[i] = bgFolder + string("\\") + basenameList[i];
			fgSiftList[i] = fgFolder + string("\\") + basenameList[i].substr(0, basenameList[i].length()-4) + string(".sift");
			fgNameList[i] = fgFolder + string("\\") + basenameList[i];


			REPORT(bgSiftList[i]);
			REPORT(bgNameList[i]);
			REPORT(fgSiftList[i]);
			REPORT(fgNameList[i]);

			sfl.removeSIFTinMask(mask, bgSiftList[i]);
			sfl.removeSIFTOutofMask(mask, fgSiftList[i]);
			imwrite(bgNameList[i], img);
			imwrite(fgNameList[i], img);
			
			//tgtNameList[i] = tgtFolder + boost::filesystem::basename(imgNameList[i]) + string(".jpg");
			string tmpShowName = boost::lexical_cast<string>(i)+string(".jpg");
			REPORT(tmpShowName);
			cv::Mat tmpForShow;
			img.copyTo(tmpForShow);
			for (int k = 0; k < sfl.siftNum; k++)
			{
				if (!(sfl.ifInMask[k]))
					continue;
				cv::circle(tmpForShow, cv::Point(sfl.allFeatsAbs[k].pos.x, sfl.allFeatsAbs[k].pos.y),
					3, cv::Scalar(0, 255, 0), 2, 8, 0);
			}
			cv::imwrite(tmpShowName, tmpForShow);
		}

}

void SIFTHandle::updateSIFTfolder(string& imgFolder, string& maskFolder, string& tgtFolder, vector<SIFTFileLoader>& out_AllSFL)
{
	if (!boost::filesystem::exists(tgtFolder))
		boost::filesystem::create_directories(tgtFolder);
	vector<string> imgNameList, siftNameList, maskNameList, tgtNameList, tgtSiftList, basenameList;


	//missing code for iterate the two folders.
	//
	boost::filesystem::recursive_directory_iterator end_iter;
	for (boost::filesystem::recursive_directory_iterator iter(imgFolder); iter != end_iter; iter++)
	{
		if (!boost::filesystem::is_directory(*iter)){
			string currentImagePath = iter->path().string();
			string currentImageS = iter->path().filename().string();
			if (iter->path().extension() == string(".jpg") ||
				iter->path().extension() == string(".png")){
				imgNameList.push_back(currentImagePath);
				basenameList.push_back(iter->path().filename().string());
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;

			}
			if (iter->path().extension() == string(".sift")){
				siftNameList.push_back(currentImagePath);
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
				maskNameList.push_back(currentImagePath);
				//	cout << "cur path" << imagePaths[imagePaths.size() - 1] << endl;

			}
		}

	}
	int fileNum = imgNameList.size();
	tgtNameList.resize(fileNum);
	tgtSiftList.resize(fileNum);
	for (int i = 0; i < fileNum; i++)
	{
		cv::Mat img = cv::imread(imgNameList[i], 1);
		cv::Mat mask = cv::imread(maskNameList[i], 0);

		SIFTFileLoader sfl(siftNameList[i]);
		tgtSiftList[i] = tgtFolder + basenameList[i].substr(0, basenameList[i].length() - 4) + string(".sift");
		tgtNameList[i] = tgtFolder + basenameList[i];

		sfl.removeSIFTOutofMask(mask, tgtSiftList[i]);

		out_AllSFL.push_back(sfl);

		imwrite(tgtNameList[i], img);
		tgtNameList[i] = tgtFolder + boost::filesystem::basename(imgNameList[i]) + string(".jpg");
		string tmpShowName = boost::lexical_cast<string>(i)+string(".jpg");
		cv::Mat tmpForShow;
		img.copyTo(tmpForShow);
		for (int k = 0; k < sfl.siftNum; k++)
		{
			if (!(sfl.ifInMask[k]))
				continue;
			cv::circle(tmpForShow, cv::Point(sfl.allFeatsAbs[k].pos.x, sfl.allFeatsAbs[k].pos.y),
				3, cv::Scalar(0, 255, 0), 2, 8, 0);
		}
		cv::imwrite(tmpShowName, tmpForShow);
	}

}
