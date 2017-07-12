#include "FgStitcher.h"
#include "Logger.h"
#include "Configer.h"

#define NODE_NOT_EXIST 0
#define NODE_MUST_LEFT 94
#define NODE_MUST_RIGHT 170
#define NODE_UNKNOWN 255

void FgStitcher::loadImageSeqs()
{
	vector<cv::Mat>& out_imgsL = fgLeftSeq;
	vector<cv::Mat>& out_imgsR = fgRightSeq;
	string baseFolder, folder, No, Cmvs;
	Configer::getConfiger()->getString("input", "baseFolder", baseFolder);
	Configer::getConfiger()->getString("input", "folder", folder);
	Configer::getConfiger()->getString("input", "No", No);
	Configer::getConfiger()->getString("input", "Cmvs", Cmvs);

	warpedFolderL = baseFolder + folder + string("\\") + No + string("\\out\\warpedFG1\\");

	warpedFolderR = baseFolder + folder + string("\\") + No + string("\\out\\warpedFG2\\"); 

	warpedFolderBG = baseFolder + folder + string("\\") + No + string("\\out\\warpedBG\\");
	
	vector<cv::Mat> imgsL, imgsR;

	int processNum = 60;
	int refineStart = 20;
	int maxRefine = 15;
	float refineStep = float(maxRefine) / float(refineStart);

	Mat element = getStructuringElement(MORPH_ELLIPSE,
		Size(3, 3),
		Point(1, 1));
	for (int k = 0; k < processNum; k++)
	{
		string nameL = warpedFolderL + string("img") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameR = warpedFolderR + string("img") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameLM = warpedFolderL + string("mask") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameRM = warpedFolderR + string("mask") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameLBG = warpedFolderBG + string("left_img") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameRBG = warpedFolderBG + string("right_img") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameLBGM = warpedFolderBG + string("left_mask") + boost::lexical_cast<string>(k)+string(".jpg");
		string nameRBGM = warpedFolderBG + string("right_mask") + boost::lexical_cast<string>(k)+string(".jpg");

		cv::Mat imgL = cv::imread(nameL, 1);
		cv::Mat imgR = cv::imread(nameR, 1);
		cv::Mat maskL = cv::imread(nameLM, 0);
		cv::Mat maskR = cv::imread(nameRM, 0);
		cv::Mat bgimgL = cv::imread(nameLBG, 1);
		cv::Mat bgimgR = cv::imread(nameRBG, 1);
		cv::Mat bgimgLM = cv::imread(nameLBGM, 0);
		cv::Mat bgimgRM = cv::imread(nameRBGM, 0);

		cv::Mat refMaskL, refMaskR;
		cv::Size smallSize(imgL.cols / 3, imgL.rows / 3);
		cv::resize(imgL, imgL, smallSize);

		cv::resize(imgR, imgR, smallSize);

		cv::resize(maskL, maskL, smallSize);
		
		cv::resize(maskR, maskR, smallSize);

		cv::resize(bgimgL, bgimgL, smallSize);

		cv::resize(bgimgR, bgimgR, smallSize);


		cv::resize(bgimgLM, bgimgLM, smallSize);

		cv::resize(bgimgRM, bgimgRM, smallSize);

		maskL.copyTo(refMaskL); maskR.copyTo(refMaskR);

		if (k > refineStart)
		{
			int ErodeTime = int(float(abs(k - refineStart)) * refineStep + 0.5);
			cv::erode(maskR, refMaskR, element, cv::Point(-1, -1), ErodeTime);
		}

		if (k < refineStart)
		{
			int ErodeTime = int(float(abs(k - (processNum - refineStart))) * refineStep + 0.5);
			cv::erode(maskL, refMaskL, element, cv::Point(-1, -1), ErodeTime);
		}
	/*	
		///ÅòÕÍ²Ù×÷
		dilate(maskL, maskL, element);
	*/	

		cv::Mat nodeID;
		cv::Mat nodeType;
		nodeID.create(cv::Size(maskR.cols, maskR.rows), CV_32FC1);
		nodeType.create(cv::Size(maskR.cols, maskR.rows), CV_8UC1);
		nodeID.setTo(-1);
		nodeType.setTo(0);
		out_imgsL.push_back(imgL);
		out_imgsR.push_back(imgR);

		fgMskLeftSeq.push_back(maskL);
		fgMskRightSeq.push_back(maskR);
		refine_fgMskLeftSeq.push_back(refMaskL);
		refine_fgMskRightSeq.push_back(refMaskR);
		nodeIDSeq1F.push_back(nodeID);
		nodeTypeSeq1U.push_back(nodeType);
		
		bgLeftSeq.push_back(bgimgL);
		bgRightSeq.push_back(bgimgR);
		bgMskLeftSeq.push_back(bgimgLM);
		bgMskRightSeq.push_back(bgimgRM);
	}
}

void getEnergyMap(vector<cv::Mat>& imgsL, vector<cv::Mat>& imgsR, vector<cv::Mat>& out_energy)
{
	out_energy.resize(imgsL.size());
	//The simplest one, just subscribe them
	for (int i = 0; i < imgsR.size(); i++)
	{
		cv::Mat tDiff;
		cv::Mat& diff = out_energy[i];
		diff.create(cv::Size(imgsR[i].cols, imgsR[i].rows), CV_32FC1);
		imgsL[i].convertTo(imgsL[i], CV_32FC3);
		imgsR[i].convertTo(imgsR[i], CV_32FC3);
		tDiff = imgsL[i] - imgsR[i];
		for (int r = 0; r < tDiff.rows; r++)
		{
			for (int c = 0; c < tDiff.cols; c++)
			{
				diff.ptr<float>(r)[c]
					= abs(tDiff.ptr<float>(r)[c * 3]) + abs(tDiff.ptr<float>(r)[c * 3 + 1])
					+ abs(tDiff.ptr<float>(r)[c * 3 + 2]);
			}
		}

		cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0.5, 0.5, 4);
	//	out_energy.push_back(diff);

	}
}

void FgStitcher::testAllFrame()
{
	vector<cv::Mat> energyMaps;
	getEnergyMap(fgLeftSeq, fgRightSeq, energyMaps);
	int h = fgLeftSeq[0].rows;
	int w = fgLeftSeq[0].cols;
	int length = energyMaps.size();
	myGraph = new GraphType(h*w*length, 3 * h*w*length);
//	GraphType::node_id currNodeId = myGraph->add_node(imgL.rows * imgR.cols);
	//Get all the valid nodes, add one by one in valid pixels
	int curNodeId = 0;
	for (int i = 0; i < length; i++)
	{

		cv::Mat& lmask = fgMskLeftSeq[i];
		cv::Mat& rmask = fgMskRightSeq[i];
		cv::Mat& ref_lmask = refine_fgMskLeftSeq[i];
		cv::Mat& ref_rmask = refine_fgMskRightSeq[i];
		cv::Mat& limg = fgLeftSeq[i];
		cv::Mat& rimg = fgRightSeq[i];
		cv::Mat& nodeID = nodeIDSeq1F[i];
		cv::Mat& nodeType = nodeTypeSeq1U[i];

/*
		cv::imshow("ref_L", ref_lmask);
		cv::imshow("ref_R", ref_rmask);
		cv::waitKey(0);*/
		//For each frame, get the valid pixels from both masks as the nodes.
		//This loop only get the nodes, then get the edges in the other separated loop
		//If the mask value is valid in both masks, it will be unknown, if left 1 right 0, then make it to a s/t point, like the first/end frame
		for (int r = 0; r < h; r++)
		{
			for (int c = 0; c < w; c++)
			{
				/*if ((lmask.ptr<uchar>(r)[c] > 100 && lmask.ptr<uchar>(r)[c] < 250)
					|| (rmask.ptr<uchar>(r)[c] > 100 && rmask.ptr<uchar>(r)[c] < 250))*/
				if ((ref_lmask.ptr<uchar>(r)[c] > 100 && ref_lmask.ptr<uchar>(r)[c] < 250)
					|| (ref_rmask.ptr<uchar>(r)[c] > 100 && ref_rmask.ptr<uchar>(r)[c] < 250))
				{
					myGraph->add_node();
					nodeType.ptr<uchar>(r)[c] = NODE_UNKNOWN;

					if (i == 0)
					{
						myGraph->add_tweights(curNodeId, (int)ceil(INT32_CONST * HARD_CONSTRAINT_CONST + 0.5), 0);
						nodeType.ptr<uchar>(r)[c] = NODE_MUST_LEFT;

					}
					if (i == length - 1)
					{
						myGraph->add_tweights(curNodeId, 0, (int)ceil(INT32_CONST * HARD_CONSTRAINT_CONST + 0.5));
						nodeType.ptr<uchar>(r)[c] = NODE_MUST_RIGHT;
					}

					if (lmask.ptr<uchar>(r)[c] < 10 
						&& (ref_rmask.ptr<uchar>(r)[c] > 100 && ref_rmask.ptr<uchar>(r)[c] < 250))
					{
						myGraph->add_tweights(curNodeId, (int)ceil(INT32_CONST * HARD_CONSTRAINT_CONST + 0.5), 0);
						nodeType.ptr<uchar>(r)[c] = NODE_MUST_RIGHT;
					}
					if ((ref_lmask.ptr<uchar>(r)[c] > 100 && ref_lmask.ptr<uchar>(r)[c] < 250)
						&& rmask.ptr<uchar>(r)[c] < 10)
					{
						myGraph->add_tweights(curNodeId, 0, (int)ceil(INT32_CONST * HARD_CONSTRAINT_CONST + 0.5));
						nodeType.ptr<uchar>(r)[c] = NODE_MUST_LEFT;
					}


					nodeID.ptr<float>(r)[c] = curNodeId;
					curNodeId++;
				}
				
			}
		}
	}
	int totalNodes = curNodeId;
	//Initial it
	curNodeId = 0;
	for (int i = 1; i < length; i++)
	{

		cv::Mat& lmask = fgMskLeftSeq[i];
		cv::Mat& rmask = fgMskRightSeq[i];
		cv::Mat& limg = fgLeftSeq[i];
		cv::Mat& rimg = fgRightSeq[i];
		cv::Mat& nodeID = nodeIDSeq1F[i];
		cv::Mat& nodeType = nodeTypeSeq1U[i];
		cv::Mat& prev_lmask = fgMskLeftSeq[i-1];
		cv::Mat& prev_rmask = fgMskRightSeq[i - 1];
		cv::Mat& prev_nodeID = nodeIDSeq1F[i - 1];
		cv::Mat& prev_nodeType = nodeTypeSeq1U[i - 1];

		cv::Mat& prev_energyMap = energyMaps[i - 1];
		cv::Mat& energyMap = energyMaps[i];
		//For each frame, get the valid pixels from both masks as the nodes.
		//This loop only get the nodes, then get the edges in the other separated loop
		//If the mask value is valid in both masks, it will be unknown, if left 1 right 0, then make it to a s/t point, like the first/end frame
		for (int r = 0; r < h; r++)
		{
			for (int c = 0; c < w; c++)
			{
				int curNodeId = nodeID.ptr<float>(r)[c];
				if (curNodeId < 0)
					continue;
				//Add the temporal links
				//Decide by the types of the node and its previous node:

				int curNodeType = nodeType.ptr<uchar>(r)[c];
				int lastNodeType = prev_nodeType.ptr<uchar>(r)[c];
				int lastNodeId = prev_nodeID.ptr<float>(r)[c];
				if (lastNodeId < 0);
				else
				{
					int v = 0.58* abs(energyMap.ptr<float>(r)[c] + prev_energyMap.ptr<float>(r)[c]);
					int edgeCapacity = (int)ceil(INT32_CONST*v + 0.5);
					if (curNodeType == NODE_UNKNOWN)
						myGraph->add_edge(curNodeId, lastNodeId, edgeCapacity, edgeCapacity);
					//If current node has to be SINK/SOURCE, then we do not have to link them, unless the last node is unknown
					else if (lastNodeType == NODE_UNKNOWN)
						myGraph->add_edge(curNodeId, lastNodeId, edgeCapacity, edgeCapacity);
				}


				//Add the spatial links
				for (int DisY = -NEIGHBORHOOD; DisY <= NEIGHBORHOOD; DisY++)
				{
					int curRow = r + DisY;
					// outside the border - skip
					if (curRow < 0 || curRow >= h)
						continue;

					float v = energyMap.ptr<float>(r)[c];
					for (int DisX = 0; DisX <= NEIGHBORHOOD; DisX++)
					{
						int curCol = c + DisX;

						// outside the border - skip
						if (curCol < 0 || curCol >= w)
							continue;

						// same pixel - skip
						// down pointed edge, this edge will be counted as an up edge for the other pixel
						if (DisY >= 0 && DisX == 0)
							continue;

						// diagonal exceed the radius - skip
						if ((DisY*DisY + DisX*DisX) > NEIGHBORHOOD*NEIGHBORHOOD)
							continue;

						// this is the node id for the neighbor
						GraphType::node_id nNodeId = nodeID.ptr<float>(curRow)[curCol];
						
						// Not a valid node, skip
						if (nNodeId < 0)
							continue;
						
						float nV = energyMap.ptr<float>(curRow)[curCol];
						//   ||I_p - I_q||^2  /   2 * sigma^2
						//	float currEdgeStrength = 
						//float currEdgeStrength = 0;
						//	float currDist = sqrt((float)si*(float)si + (float)sj*(float)sj);
						//		float nV = (float)diff.ptr<float>(i + si)[j + sj];
						// this is the edge between the current two pixels (i,j) and (i+si, j+sj)
						float currEdgeStrength = 0.5 * abs(v + nV);
						int edgeCapacity = /* capacities */ (int)ceil(INT32_CONST*currEdgeStrength + 0.5);
						//edgeCapacity = 0;
						myGraph->add_edge(curNodeId, nNodeId, edgeCapacity, edgeCapacity);

					}
				} // End spatial links , 4 direction

		//			nodeID.ptr<float>(r)[c] == curNodeId;
		//			curNodeId++;

			} // End each row
		}// End for one pixel
	}//End for one image


	int flow = myGraph->maxflow();
	cout << "done maxflow: " << flow << endl;

	//myGraph->what_segment((GraphType::node_id)i) == GraphType::SOURCE
	for (int i = 0; i < length; i++)
	{
		cv::Mat& limg = fgLeftSeq[i];
		cv::Mat& rimg = fgRightSeq[i];
		cv::Mat& bglimg = bgLeftSeq[i];
		cv::Mat& bgrimg = bgRightSeq[i];

		cv::Mat& bglMsk = bgMskLeftSeq[i];
		cv::Mat& bgrMsk = bgMskRightSeq[i];
		cv::Mat& nodeID = nodeIDSeq1F[i];

		cv::Mat& lmask = fgMskLeftSeq[i];
		cv::Mat& rmask = fgMskRightSeq[i];

		cv::Mat stitchedImg;
		cv::Mat stitchedMask;
		stitchedMask.create(cv::Size(limg.cols, limg.rows), CV_8UC1);
		stitchedMask.setTo(0);
		limg.copyTo(stitchedImg);
		for (int r = 0; r < h; r++)
		{
			for (int c = 0; c < w; c++)
			{
				int curNodeId = nodeID.ptr<float>(r)[c];
				stitchedImg.ptr<float>(r)[c * 3] = bglimg.ptr<uchar>(r)[c * 3];
				stitchedImg.ptr<float>(r)[c * 3 + 1] = bglimg.ptr<uchar>(r)[c * 3 + 1];
				stitchedImg.ptr<float>(r)[c * 3 + 2] = bglimg.ptr<uchar>(r)[c * 3 + 2];

				if (bglMsk.ptr<uchar>(r)[c] >= 250)
				{
					if (bgrMsk.ptr<uchar>(r)[c] < 10)
					{
						stitchedImg.ptr<float>(r)[c * 3] = bgrimg.ptr<uchar>(r)[c * 3];
						stitchedImg.ptr<float>(r)[c * 3 + 1] = bgrimg.ptr<uchar>(r)[c * 3 + 1];
						stitchedImg.ptr<float>(r)[c * 3 + 2] = bgrimg.ptr<uchar>(r)[c * 3 + 2];
					}
				}	
				if (bglMsk.ptr<uchar>(r)[c] > 10 && bglMsk.ptr<uchar>(r)[c] < 250)				
				{
					if (bgrMsk.ptr<uchar>(r)[c] < 10)
					{
						stitchedImg.ptr<float>(r)[c * 3] = bgrimg.ptr<uchar>(r)[c * 3];
						stitchedImg.ptr<float>(r)[c * 3 + 1] = bgrimg.ptr<uchar>(r)[c * 3 + 1];
						stitchedImg.ptr<float>(r)[c * 3 + 2] = bgrimg.ptr<uchar>(r)[c * 3 + 2];
					}
				}
				if (curNodeId < 0)
					continue;
	
				if (myGraph->what_segment(curNodeId) == GraphType::SOURCE)
				{

					stitchedMask.ptr<uchar>(r)[c] = 128;
					stitchedImg.ptr<float>(r)[c * 3] = rimg.ptr<float>(r)[c * 3];
					stitchedImg.ptr<float>(r)[c * 3 + 1] = rimg.ptr<float>(r)[c * 3 + 1];
					stitchedImg.ptr<float>(r)[c * 3 + 2] = rimg.ptr<float>(r)[c * 3 + 2];
				}
				else
				{
					stitchedMask.ptr<uchar>(r)[c] = 255;
					stitchedImg.ptr<float>(r)[c * 3] = limg.ptr<float>(r)[c * 3];
					stitchedImg.ptr<float>(r)[c * 3 + 1] = limg.ptr<float>(r)[c * 3 + 1];
					stitchedImg.ptr<float>(r)[c * 3 + 2] = limg.ptr<float>(r)[c * 3 + 2];
				}
			}
		}
		string x = string("stitched") + boost::lexical_cast<string>(i)+string(".jpg");
		string x2 = string("stitched_mask") + boost::lexical_cast<string>(i)+string(".jpg");
		string x3 = string("stitched_Initial") + boost::lexical_cast<string>(i)+string(".jpg");
		cv::imwrite(x, stitchedImg);
		cv::imwrite(x2, stitchedMask);
		cv::imwrite(x3, nodeTypeSeq1U[i]);
	}
}

void FgStitcher::testTwoFrame()
{
	cv::Mat imgL = imread("img1.jpg", 1);
	cv::Mat imgR = imread("img2.jpg", 1);
	cv::Mat imgL3C;
	imgL.copyTo(imgL3C);
	imgL.convertTo(imgL, CV_32FC3);
	imgR.convertTo(imgR, CV_32FC3);
	cv::Mat imgDiff = imgL - imgR;
	cv::Mat diff;
	diff.create(cv::Size(imgDiff.cols, imgDiff.rows), CV_32FC1);
	for (int i = 0; i < imgDiff.rows; i++)
	{
		for (int j = 0; j < imgDiff.cols; j++)
		{
			diff.ptr<float>(i)[j] =
				abs(imgDiff.ptr<float>(i)[j * 3]) + abs(imgDiff.ptr<float>(i)[j * 3 + 1]) + abs(imgDiff.ptr<float>(i)[j * 3 + 2]);
		}
	}
	cv::GaussianBlur(diff, diff, cv::Size(5, 5), 0.5, 0.5, 4);
	diff = 0.01* diff;
	cv::imshow("diff", diff);
	cv::waitKey(0);
	myGraph = new GraphType(imgL.rows*imgL.cols, 4 * imgL.rows*imgL.cols);
	GraphType::node_id currNodeId = myGraph->add_node(imgL.rows * imgR.cols);
	for (int i = 0; i < imgDiff.rows; i++)
	{
		for (int j = 0; j < imgDiff.cols; j++)
		{
			GraphType::node_id currNodeId = i * imgDiff.cols + j;

			if (j >= imgDiff.cols - 200)
				myGraph->add_tweights(currNodeId, 0, (int)ceil(INT32_CONST * HARD_CONSTRAINT_CONST + 0.5));

			if (j <= 200)
				myGraph->add_tweights(currNodeId, (int)ceil(INT32_CONST * HARD_CONSTRAINT_CONST + 0.5), 0);


			for (int si = -NEIGHBORHOOD; si <= NEIGHBORHOOD; si++)
			{
				int ni = i + si;
				// outside the border - skip
				if (ni < 0 || ni >= diff.rows)
					continue;

				float v = diff.ptr<float>(i)[j];
				for (int sj = 0; sj <= NEIGHBORHOOD; sj++)
				{
					int nj = j + sj;
					// outside the border - skip
					if (nj < 0 || nj >= diff.cols)
						continue;

					// same pixel - skip
					// down pointed edge, this edge will be counted as an up edge for the other pixel
					if (si >= 0 && sj == 0)
						continue;

					// diagonal exceed the radius - skip
					if ((si*si + sj*sj) > NEIGHBORHOOD*NEIGHBORHOOD)
						continue;


					// this is the node id for the neighbor
					GraphType::node_id nNodeId = (i + si) * diff.cols + (j + sj);

					float nV = diff.ptr<float>(i + si)[j + sj];
					//   ||I_p - I_q||^2  /   2 * sigma^2
				//	float currEdgeStrength = 
						//float currEdgeStrength = 0;
				//	float currDist = sqrt((float)si*(float)si + (float)sj*(float)sj);
			//		float nV = (float)diff.ptr<float>(i + si)[j + sj];
					// this is the edge between the current two pixels (i,j) and (i+si, j+sj)
					float currEdgeStrength = abs(v - nV);
					int edgeCapacity = /* capacities */ (int)ceil(INT32_CONST*currEdgeStrength + 0.5);
					//edgeCapacity = 0;
					myGraph->add_edge(currNodeId, nNodeId, edgeCapacity, edgeCapacity);

				}
			}

		}
	}
	int flow = myGraph->maxflow();
	cout << "done maxflow: " << flow << endl;

	cv::Mat segMask, segShowImg, tempShowImg;
	imgL3C.copyTo(tempShowImg);
	segMask.create(imgL.rows, imgL.cols, CV_8UC1);
	segShowImg.create(imgL.rows, imgL.cols, CV_8UC3);

	for (int i = 0; i < imgL.rows * imgL.cols; i++)
	{
		int r = i / imgL.cols;
		int c = i % imgL.cols;
		// if it is foreground - color blue
		if (myGraph->what_segment((GraphType::node_id)i) == GraphType::SOURCE)
		{
			segMask.at<uchar>(r, c) = 255;
			(uchar)segShowImg.at<Vec3b>(r, c)[2] = 200;
		}
		// if it is background - color red
		else
		{
			segMask.at<uchar>(r, c) = 0;
			(uchar)segShowImg.at<Vec3b>(r, c)[0] = 200;
			imgL3C.ptr<uchar>(r)[c * 3] = imgR.ptr<float>(r)[c * 3];
			imgL3C.ptr<uchar>(r)[c * 3 + 1] = imgR.ptr<float>(r)[c * 3 + 1];
			imgL3C.ptr<uchar>(r)[c * 3 + 2] = imgR.ptr<float>(r)[c * 3 + 2];
		}
	}
	segShowImg = 0.5*segShowImg + 0.5*tempShowImg;
	imshow("Segmentation Mask", segMask);
	imshow("Segmentation Image", segShowImg);
	imshow("Stitched Image", imgL3C);
	cv::waitKey(0);

}

FgStitcher::FgStitcher()
{
}


FgStitcher::~FgStitcher()
{
}
