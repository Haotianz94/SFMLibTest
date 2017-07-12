#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <memory.h>
#include <time.h>
#include <iostream>
#include "Geometry/Include/EuclideanGeometry.h"
#include "Geometry/Include/SE3Geometry.h"
#include "Geometry/Include/GrassmannGeometry.h"
#include "Geometry/Include/EssentialGeometry.h"
#include "Geometry/Include/DTIAffineInvariantGeometry.h"

#include "MeanShiftLib/Include/VectorPointSet.h"
#include "MeanShiftLib/Include/EssentialPointSet.h"

#include "MeanShiftLib/Include/MeanShift.h"
using namespace std;
int mainxxx(int argc,char** argv){

	//////////////////////////////////////////////////////////////
	//															//
	//						MEAN SHIFT CODE						//
	//															//
	//////////////////////////////////////////////////////////////

/*
	int rows, cols;
	float *data;

	FILE* fp = fopen(argv[2], "r");
	fscanf(fp, "%d", &rows);
	fscanf(fp, "%d", &cols);
	data = new float[rows * cols];
	for(int rc = 0; rc < rows * cols; rc++)
		fscanf(fp, "%f", data + rc);
	fclose(fp);

	CMeanShift<float> ms;
	ms.setBandwidth(atof(argv[3]));

	///////////////////// EUCLIDEAN MEAN SHIFT /////////////////////
	if(strcmp("-euc", argv[1]) == 0){

		CEuclideanGeometry<float> geom(cols);
		CVectorPointSet<float> dataPoints(cols, rows, data);
		CVectorPointSet<float> unprunedModes(cols, rows);

		ms.doMeanShift(geom, dataPoints, unprunedModes);
		unprunedModes.write("unpruned.txt");
		CVectorPointSet<float> prunedModes(cols, 100);
		ms.pruneModes(geom, unprunedModes, prunedModes, 10, 1.0);

		double kernelDensities[100];
		ms.getKernelDensities(geom, dataPoints, prunedModes, kernelDensities);

		prunedModes.write("pruned.txt", kernelDensities);

	}

	///////////////////// GRASSMANN MEAN SHIFT ////////////////////
	if(strcmp("-gmn", argv[1]) == 0){

		int n = atoi(argv[4]);
		int k = atoi(argv[5]);
		CGrassmannGeometry<float> geom(n, k);
		CVectorPointSet<float> dataPoints(n * k, rows, data);
		CVectorPointSet<float> unprunedModes(n * k, rows);

		ms.doMeanShift(geom, dataPoints, unprunedModes);

		CVectorPointSet<float> prunedModes(n * k, 100);
		ms.pruneModes(geom, unprunedModes, prunedModes, 10, 1.0);

		double kernelDensities[100];
		ms.getKernelDensities(geom, dataPoints, prunedModes, kernelDensities);

		prunedModes.write("pruned.txt", kernelDensities);

	}

	///////////////////// ESSENTIAL MEAN SHIFT ////////////////////
	if(strcmp("-ess", argv[1]) == 0){

		CEssentialGeometry<float> geom;
		CEssentialPointSet<float> dataPoints(rows, data);
		CEssentialPointSet<float> unprunedModes(rows);

		ms.doMeanShift(geom, dataPoints, unprunedModes);

		CEssentialPointSet<float> prunedModes(100);
		ms.pruneModes(geom, unprunedModes, prunedModes, 10, 1.0);

		double kernelDensities[100];
		ms.getKernelDensities(geom, dataPoints, prunedModes, kernelDensities);

		prunedModes.write("pruned.txt", kernelDensities);

	}

	/////////////////////// SE(3) MEAN SHIFT //////////////////////
	if(strcmp("-se3", argv[1]) == 0){

		float transScale;
		if(argc == 4)
			transScale = 25;
		else
			transScale = atof(argv[4]);
		CSE3Geometry<float> geom(transScale);
		CVectorPointSet<float> dataPoints(16, rows, data);
		CVectorPointSet<float> unprunedModes(16, rows);

		ms.doMeanShift(geom, dataPoints, unprunedModes);

		CVectorPointSet<float> prunedModes(16, 100);
		ms.pruneModes(geom, unprunedModes, prunedModes, 10, 1.0);

		double kernelDensities[100];
		ms.getKernelDensities(geom, dataPoints, prunedModes, kernelDensities);

		prunedModes.write("pruned.txt", kernelDensities);

	}

	delete [] data;
*/

	//////////////////////////////////////////////////////////////
	//															//
	//							RETURN							//
	//															//
	//////////////////////////////////////////////////////////////

	return 0;

}