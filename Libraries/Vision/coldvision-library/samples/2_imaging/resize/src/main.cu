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
#include "timer.h"
#include "resize.cuh"

using namespace std;
using namespace cv;

void processUsingOpenCvCpu(std::string nput_file, std::string output_file);
void processUsingOpenCvGpu(std::string input_file, std::string output_file);
void processUsingCuda(std::string input_file, std::string output_file);

int main(int argc, char **argv) {

//#if defined(__linux__)
//	setenv ("DISPLAY", ":0", 0);
//#endif

	const string input_file = argc >= 2 ? argv[1] : "../data/simple_room-wallpaper-4096x3072.jpg";
	const string output_file_OpenCvCpu = argc >= 3 ? argv[2] : "../data/output_1_4th_OpenCvCpu.jpg";
	const string output_file_OpenCvGpu = argc >= 4 ? argv[3] : "../data/output_1_4th_OpenCvGpu.jpg";
	const string output_file_Cuda = argc >= 5 ? argv[2] : "../data/output_1_4th_Cuda.jpg";

	for (int i=0; i<10; ++i) {
		processUsingOpenCvCpu(input_file, output_file_OpenCvCpu);
		processUsingOpenCvGpu(input_file, output_file_OpenCvGpu);
		processUsingCuda(input_file, output_file_Cuda);
	}

	return 0;
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

	GpuTimer timer;
	timer.Start();
	resize(input, output, Size(), .25, 0.25, CV_INTER_AREA); // downscale 4x on both x and y

	timer.Stop();
	printf("OpenCv Cpu code ran in: %f msecs.\n", timer.Elapsed());

	imwrite(output_file, output);
}

void processUsingOpenCvGpu(std::string input_file, std::string output_file) {
	//Read input image from the disk
	Mat inputCpu = imread(input_file,CV_LOAD_IMAGE_COLOR);
	cuda::GpuMat input (inputCpu);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	//Create output image
	cuda::GpuMat output;

	GpuTimer timer;
	timer.Start();

	cuda::resize(input, output, Size(), .25, 0.25, CV_INTER_AREA); // downscale 4x on both x and y

	timer.Stop();
	printf("OpenCv Gpu code ran in: %f msecs.\n", timer.Elapsed());

	Mat outputCpu;
	output.download(outputCpu);
	imwrite(output_file, outputCpu);

	input.release();
	output.release();
}

void processUsingCuda(std::string input_file, std::string output_file) {
	//Read input image from the disk
	cv::Mat input = cv::imread(input_file,CV_LOAD_IMAGE_UNCHANGED);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	//Create output image
	Size newSize( input.size().width / 4, input.size().height / 4 ); // downscale 4x on both x and y
	Mat output (newSize, input.type());

	downscaleCuda(input, output);

	imwrite(output_file, output);
}
