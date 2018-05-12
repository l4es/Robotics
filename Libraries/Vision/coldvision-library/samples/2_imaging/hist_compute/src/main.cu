/* *
 * Copyright 1993-2012 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudawarping.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "timer.h"
#include "hist_compute.cuh"

using namespace std;
using namespace cv;

void processUsingOpenCvCpu(std::string nput_file, std::string output_file);
void processUsingOpenCvGpu(std::string input_file, std::string output_file);
void processUsingCuda(std::string input_file, std::string output_file);

int main(int argc, char **argv) {

	const string input_file = argc >= 2 ? argv[1] : "../data/simple_room-wallpaper-4096x3072.jpg";
	const string output_file_OpenCvCpu = argc >= 3 ? argv[2] : "../data/output_OpenCvCpu.jpg";
	const string output_file_OpenCvGpu = argc >= 4 ? argv[3] : "../data/output_OpenCvGpu.jpg";
	const string output_file_Cuda = argc >= 5 ? argv[2] : "../data/output_Cuda.jpg";

	for (int i=0; i<5; ++i) {
		processUsingOpenCvCpu(input_file, output_file_OpenCvCpu);
		processUsingOpenCvGpu(input_file, output_file_OpenCvGpu);
		processUsingCuda(input_file, output_file_Cuda);
	}

	return 0;
}

template <typename Type>
Mat imHist(Mat hist, float scaleX=1, float scaleY=1){
	double maxVal=0;
	minMaxLoc(hist, 0, &maxVal, 0, 0);
	int rows = 64; //default height size
	int cols = hist.rows; //get the width size from the histogram
	Mat histImg = Mat::zeros(rows*scaleX, cols*scaleY, CV_8UC1);
	//for each bin
	for(int i=0;i<cols-1;i++) {
		Type histValue = hist.at<Type>(i,0);
		Type nextValue = hist.at<Type>(i+1,0);
		Point pt1 = Point(i*scaleX, rows*scaleY);
		Point pt2 = Point(i*scaleX+scaleX, rows*scaleY);
		Point pt3 = Point(i*scaleX+scaleX, (rows-nextValue*rows/maxVal)*scaleY);
		Point pt4 = Point(i*scaleX, (rows-nextValue*rows/maxVal)*scaleY);

		int numPts = 5;
		Point pts[] = {pt1, pt2, pt3, pt4, pt1};

		fillConvexPoly(histImg, pts, numPts, Scalar(255,255,255));
	}
	return histImg;
}

void processUsingOpenCvCpu(std::string input_file, std::string output_file) {
	//Read input image from the disk
	Mat input = imread(input_file, CV_LOAD_IMAGE_COLOR);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	//Create output image
	Mat output;

	//Hold the histogram
	Mat hist, histImg;
	int nbins = 256; // lets hold 256 levels
	int hsize[] = { nbins }; // just one dimension
	float range[] = { 0, 256 };
	const float *ranges[] = { range };
	int chnls[] = { 0 };

	// create colors channels
	vector<Mat> colors;
	split(input, colors);

	// compute for all colors
	calcHist(&colors[0], 1, chnls, Mat(), hist, 1, hsize, ranges);
	histImg = imHist<float>(hist, 3, 3);
	//imshow("Blue", histImg);

	calcHist(&colors[1], 1, chnls, Mat(), hist, 1, hsize, ranges);
	histImg = imHist<float>(hist, 3, 3);
	//imshow("Green", histImg);

	GpuTimer timer;
	timer.Start();
	calcHist(&colors[2], 1, chnls, Mat(), hist, 1, hsize, ranges);
	timer.Stop();
	printf("OpenCv Cpu code ran in: %f msecs.\n", timer.Elapsed());

	//cout << hist << endl;

	// create an image out of the hist to be displayed
	histImg = imHist<float>(hist, 3, 3);
	//imshow("Red", histImg);

	// show image
	//imshow("Image", input);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, histImg);
}

void processUsingOpenCvGpu(std::string input_file, std::string output_file) {
	//Read input image from the disk
	Mat inputCpu = imread(input_file,CV_LOAD_IMAGE_COLOR);
	if(inputCpu.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	//Hold the histogram
	Mat hist, histImg;

	// create colors channels
	vector<Mat> colors;
	split(inputCpu, colors);

	cuda::GpuMat colorGpu (colors[2]); // select the third color channel
	cuda::GpuMat histGpu;

	// compute histogram
	GpuTimer timer;
	timer.Start();
	cv::cuda::calcHist(colorGpu, histGpu);
	timer.Stop();
	printf("OpenCv Gpu code ran in: %f msecs.\n", timer.Elapsed());

	histGpu.download(hist);
	colorGpu.release();
	histGpu.release();

	// change the form from one row (n columns) to a vector (n rows)
	hist = hist.reshape(1, 256);
	//cout << hist << endl;

	// create an image out of the hist to be displayed
	histImg = imHist<unsigned int>(hist, 3, 3);
	//imshow("red", histImg);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, histImg);
}

void processUsingCuda(std::string input_file, std::string output_file) {
	//Read input image from the disk
	cv::Mat input = cv::imread(input_file,CV_LOAD_IMAGE_UNCHANGED);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	//Hold the histogram
	Mat hist (Size(1,256), CV_32SC1);
	Mat histImg;

	// create colors channels
	vector<Mat> colors;
	split(input, colors);

	calcHistCuda(colors[2], hist);

	//cout << hist << endl;

	// create an image out of the hist to be displayed
	histImg = imHist<unsigned int>(hist, 3, 3);
	//imshow("red", histImg);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, histImg);
}
