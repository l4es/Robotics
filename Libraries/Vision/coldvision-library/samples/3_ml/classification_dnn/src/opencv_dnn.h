/*
 * opencv_dnn.h
 *
 *  Created on: Jul 29, 2016
 *      Author: claudiu
 */

#ifndef OPENCV_DNN_H_
#define OPENCV_DNN_H_

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <map>
#include <iomanip>
#include "timer.h"

using namespace cv;
using namespace cv::dnn;
using namespace std;

/*
 * Find best classes for the blob (i. e. class with maximal probability).
 *
 * @param probBlob Contains the probabilities of an image to be from a specific class, one probability for each classId.
 * @param bestMatches A container to store the best matches.
 *
 * @see http://docs.opencv.org/trunk/d5/de7/tutorial_dnn_googlenet.html
 */
void getBestMatches( cv::dnn::Blob &probBlob, std::vector < pair<double , int> >& bestMatches )
{
	// reshape the blob to 1x1000 matrix
	Mat probMat = probBlob.matRefConst().reshape(1, 1);

	// build a map of {probability, classId} in decreasing order, highest probability first
	std::multimap<double, int, greater<double> > map;
	std::multimap<double, int, greater<double> >::iterator mapIt;
	for(int i=0; i<probMat.cols; ++i)
	{
		int classId = i; // position in the array represents the classId
		double classProb = (double) probMat.at<float>(0, i); // the probMat stores its elements as float
		map.insert ( std::pair<double, int>(classProb, classId) );
	}

	// select the best matches, which are in the first positions in the sorted map
	int index = 0;
	for( mapIt = map.begin(); mapIt!=map.end(); mapIt++ )
	{
		bestMatches[index] = *mapIt; // the map and the vector have the same type: pair<double, int>
		if( ++index == bestMatches.size() )
		{
			break; // stop when first bestMatches are copied
		}
	}
}

/*
 * It loads the class IDs and names from the file system.
 *
 * @see http://docs.opencv.org/trunk/d5/de7/tutorial_dnn_googlenet.html
 */
void readClassNames(const std::string fileName, std::vector<string>& classNames)
{
	std::ifstream fp(fileName.c_str());
	if (!fp.is_open())
	{
		std::cerr << "File with classes labels not found: " << fileName << std::endl;
		exit(-1);
	}
	std::string className;
	while (!fp.eof())
	{
		std::getline(fp, className);
		if (className.length())
		{
			classNames.push_back( className.substr(className.find(' ')+1) );
		}
	}
	fp.close();
}

void loadCaffeNet(const string & modelTxt, const string & modelBin, dnn::Net & net)
{
	// try to import the Caffe model
	Ptr<dnn::Importer> importer;
	try
	{
		importer = dnn::createCaffeImporter(modelTxt, modelBin);
	}
	catch (const cv::Exception& err) //Importer can throw errors, we will catch them
	{
		std::cerr << err.msg << std::endl;
	}

	// check if errors occurred while creating the importer from the file
	if (!importer)
	{
		std::cerr << "Can't load network by using the following files: " << std::endl;
		std::cerr << "prototxt:   " << modelTxt << std::endl;
		std::cerr << "caffemodel: " << modelBin << std::endl;
		std::cerr << "bvlc_googlenet.caffemodel can be downloaded here:" << std::endl;
		std::cerr << "http://dl.caffe.berkeleyvision.org/bvlc_googlenet.caffemodel"	<< std::endl;
		exit(-1);
	}

	// Adds loaded layers into the net and sets connections between them.
	importer->populateNet(net);
	importer.release(); //We don't need importer anymore
}

void testOpenCVDnnWithCaffeModel(const string & modelTxtPath, const string & modelBinPath,
		const string & classesFilePath, const string & imageFilePath)
{
	cout << "Testing OpenCV DNN with Caffe Model, modelTxt:" << modelTxtPath << ", modelBin:" << modelBinPath << ", "<< endl
			<< "classesFile:" << classesFilePath << ", imageFile:" << imageFilePath << endl;

	// import the Caffe model
	dnn::Net net;
	loadCaffeNet(modelTxtPath, modelBinPath, net);

	// load the class IDs and names
	std::vector<string> classNames;
	readClassNames(classesFilePath, classNames);

	// load the image to be classified into the dnn::Blob format
	Mat img = imread(imageFilePath);
	if (img.empty())
	{
		std::cerr << "Can't read image from the file: " << imageFilePath << std::endl;
		exit(-1);
	}
	resize(img, img, Size(224, 224)); //GoogLeNet accepts only 224x224 RGB-images
	dnn::Blob inputBlob = dnn::Blob(img); //Convert Mat to dnn::Blob image batch

	// apply the blob on the input layer
	net.setBlob(".data", inputBlob); //set the network input

	// Start the timer
	GpuTimer timer;
	timer.Start();

	// classify the image by applying the blob on the net
	net.forward(); //compute output

	timer.Stop();

	// find the best 5 matches and their probabilities
	dnn::Blob probabilities = net.getBlob("prob"); //gather output of "prob" layer
	int numberOfMatches = 5; // find top best matches
	vector < pair<double , int> > bestMatches (numberOfMatches);
	vector < pair<double , int> >::iterator matchIt;
	getBestMatches(probabilities, bestMatches); //find the best class

	// print the best matches
	for( matchIt = bestMatches.begin(); matchIt!=bestMatches.end(); matchIt++ )
	{
		double classProb = (*matchIt).first;
		int classId = (*matchIt).second;
		std::cout << std::fixed << std::setprecision(4) << classProb * 100 << "% - \"#" << classId
				<< " " << classNames.at(classId) << "\"" << std::endl;
	}

	cout << "Classification time (msec): " << timer.Elapsed() << endl;
}

#endif /* OPENCV_DNN_H_ */
