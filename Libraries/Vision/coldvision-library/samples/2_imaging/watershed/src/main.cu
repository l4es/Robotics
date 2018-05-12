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
#include "timer.h"
#include "watershed_markers.h"
#include "watershed_markers_hist.h"
#include "watershed_power.h"
#include "watershed_order_variant.cuh"
//#include "watershed_mamba.h"

using namespace std;
using namespace cv;

void processUsingOpenCvCpuAndMarkers(std::string nput_file, std::string output_file);
void processUsingOpenCvCpuMarkersAndHist(std::string nput_file, std::string output_file);
void processUsingPowerWatershedCpu(std::string input_file, std::string output_file);
void processUsingOrderVariantWatershedCuda(std::string input_file, std::string output_file);
//void processUsingMambaWatershedCpu(std::string input_file, std::string output_file);

int main(int argc, char **argv) {

	string fileId = "0";
	const string input_file = argc >= 2 ? argv[1] : "../data/watershed_test_"+fileId+".jpg";
	const string output_file_OpenCvCpuAndMarkers = argc >= 3 ? argv[2] : "../data/watershed_out_"+fileId+"_OpenCv_Cpu_And_Markers.jpg";
	const string output_file_OpenCvCpuMarkersAndHist = argc >= 3 ? argv[2] : "../data/watershed_out_"+fileId+"_OpenCv_Cpu_Markers_And_Histograms.jpg";
	const string output_file_PowerWatershedCpu = argc >= 3 ? argv[2] : "../data/watershed_out_"+fileId+"_PowerWatershed_Cpu.jpg";
	const string output_file_OrderVariantWatershedCuda = argc >= 3 ? argv[2] : "../data/watershed_out_"+fileId+"_Order_Variant_Watershed_Cuda.jpg";
	const string output_file_Cuda = argc >= 3 ? argv[2] : "../data/watershed_out_"+fileId+"_Cuda.jpg";
	//const string output_file_MambaWatershedCpu = argc >= 3 ? argv[2] : "../data/watershed_out_"+fileId+"_Mamba_pu.jpg";

	for (int i=0; i<1; ++i) {
		std::cout<<"Processing file: "<< input_file << std::endl;
		processUsingOpenCvCpuAndMarkers(input_file, output_file_OpenCvCpuAndMarkers);
		processUsingOpenCvCpuMarkersAndHist(input_file, output_file_OpenCvCpuMarkersAndHist);
		processUsingPowerWatershedCpu(input_file, output_file_PowerWatershedCpu);
		processUsingOrderVariantWatershedCuda(input_file, output_file_OrderVariantWatershedCuda);
		//processUsingMambaWatershedCpu(input_file, output_file_MambaWatershedCpu);
	}

	return 0;
}

void processUsingOpenCvCpuAndMarkers(std::string input_file, std::string output_file) {

	//Read input image from the disk
	Mat input = imread(input_file, CV_LOAD_IMAGE_COLOR);
	if(input.empty()) 	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	//show original image
	//imshow("Original Image", input);

	GpuTimer timer;
	timer.Start();

	// display the merged segments blended with the image
	Mat wshedWithImage = watershedWithMarkers(input);

	timer.Stop();
	printf("OpenCvCpuAndMarkers code ran in: %f msecs.\n", timer.Elapsed());

	//Display the merged segments blended with the image
	//imshow("Final Result", wshedWithImage);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, wshedWithImage);
}

void processUsingOpenCvCpuMarkersAndHist(std::string input_file, std::string output_file) {

	//Read the file
	Mat image = imread(input_file, CV_LOAD_IMAGE_COLOR);

	//show original image
	//imshow("Original Image", image);

	GpuTimer timer;
	timer.Start();

	// display the merged segments blended with the image
	Mat wshedWithImage = watershedWithMarkersAndHistograms(image);

	timer.Stop();
	printf("OpenCvCpuMarkersAndHist code ran in: %f msecs.\n", timer.Elapsed());

	//Display the merged segments blended with the image
	//imshow("Final Result", wshedWithImage);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, wshedWithImage);
}

void processUsingPowerWatershedCpu(std::string input_file, std::string output_file) {

	input_file.replace(input_file.find("jpg"), 3, "ppm");
	output_file.replace(output_file.find("jpg"), 3, "ppm");
	string seeds_name = "../data/watershed_seeds_MULT.pgm";
	int algo = 2;
	bool mult = true;
	bool geod = false;

	GpuTimer timer;
	timer.Start();

	watershedPower(input_file, output_file, seeds_name, algo, mult, geod);

	timer.Stop();
	printf("PowerWatershedCPU code ran in: %f msecs.\n", timer.Elapsed());
}

void processUsingOrderVariantWatershedCuda(std::string input_file, std::string output_file) {
	//Read the file
	Mat image = imread(input_file, CV_LOAD_IMAGE_GRAYSCALE);

	// pre-process it
	GaussianBlur( image, image, Size( 5, 5 ), 0, 0 ); // add blur to remove noise
	resize(image, image, Size(), 0.5, 0.5, INTER_CUBIC);
	image.convertTo(image, CV_32FC1); // convert it to float
	normalize(image, image, 0.0f, 255.0f, NORM_MINMAX, CV_32F);

	//show original image
	//imshow("Original Image", image);

	GpuTimer timer;
	timer.Start();

	// display the merged segments blended with the image
	Mat wshedWithImage (image.size(), image.type());  // CV_32FC1
	segmentWithTextureAnalysis(image, wshedWithImage);
	timer.Stop();
	printf("OrderVariantWatershedCuda code ran in: %f msecs.\n", timer.Elapsed());

	Mat normWshedWithImage (image.size(), CV_8UC1);
	normalize(wshedWithImage, normWshedWithImage, 0, 255, NORM_MINMAX, CV_8UC1);

	//Display the merged segments blended with the image
	//imshow("Final Result", wshedWithImage);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, normWshedWithImage);
}

/*
// NOT ported to C++
void processUsingMambaWatershedCpu(std::string input_file, std::string output_file) {

	//Read the file
	Mat image = imread(input_file, CV_LOAD_IMAGE_COLOR);
	cv::cvtColor(image, image, CV_BGR2BGRA, 4);

	//show original image
	//imshow("Original Image", image);

	GpuTimer timer;
	timer.Start();

	// display the merged segments blended with the image
	Mat wshedWithImage = watershedMamba(image);

	timer.Stop();
	printf("WatershedMambaCPU code ran in: %f msecs.\n", timer.Elapsed());

	//Display the merged segments blended with the image
	//imshow("Final Result", wshedWithImage);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, wshedWithImage);
}
 */
