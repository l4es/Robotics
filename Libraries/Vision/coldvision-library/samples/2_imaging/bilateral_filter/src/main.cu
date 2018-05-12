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
#include "opencv2/cudaimgproc.hpp"
#include "timer.h"
#include "bilateral_filter_cpu.h"
#include "bilateral_filter_cuda.cuh"

using namespace std;
using namespace cv;

void processUsingOpenCvCpu(std::string nput_file, std::string output_file);
void processUsingOpenCvGpu(std::string input_file, std::string output_file);
void processUsingCpu(std::string input_file, std::string output_file);
void processUsingCuda(std::string input_file, std::string output_file);

int main(int argc, char **argv) {

	//#if defined(__linux__)
	//	setenv ("DISPLAY", ":0", 0);
	//#endif

	const string input_file = argc >= 2 ? argv[1] : "../data/simple_room-wallpaper-4096x3072.jpg";
	const string output_file_OpenCvCpu = argc >= 3 ? argv[2] : "../data/output_OpenCvCpu.jpg";
	const string output_file_OpenCvGpu = argc >= 4 ? argv[3] : "../data/output_OpenCvGpu.jpg";
	const string output_file_Cpu = argc >= 5 ? argv[4] : "../data/output_Cpu.jpg";
	const string output_file_Cuda = argc >= 6 ? argv[5] : "../data/output_Cuda.jpg";

	for (int i=0; i<10; ++i) {
		processUsingOpenCvCpu(input_file, output_file_OpenCvCpu);
		processUsingOpenCvGpu(input_file, output_file_OpenCvGpu);
		processUsingCpu(input_file, output_file_Cpu);
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
	bilateralFilter(input, output, 5, 150, 150);

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

	cuda::bilateralFilter(input, output, 5, 150, 150);

	timer.Stop();
	printf("OpenCv Gpu code ran in: %f msecs.\n", timer.Elapsed());

	Mat outputCpu;
	output.download(outputCpu);
	imwrite(output_file, outputCpu);

	input.release();
	output.release();
}

void processUsingCpu(std::string input_file, std::string output_file) {
	//Read input image from the disk
	cv::Mat input = cv::imread(input_file,IMREAD_UNCHANGED);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	// convert from char(0-255) BGR to float (0.0-0.1) RGBA
	Mat inputRGBA;
	cvtColor(input, inputRGBA, CV_BGR2RGBA, 4);
	inputRGBA.convertTo(inputRGBA, CV_32FC4);
	inputRGBA /= 255;

	//Create output image
	Mat output (input.size(), inputRGBA.type());

	const float euclidean_delta = 1.0f;
	const int filter_radius = 5;

	GpuTimer timer;
	timer.Start();

	bilateralFilterCpu((float4*) inputRGBA.ptr<float4>(),
			(float4*) output.ptr<float4>(),
			euclidean_delta,
			inputRGBA.cols, inputRGBA.rows,
			filter_radius);

	timer.Stop();
	printf("Own CPU code ran in: %f msecs.\n", timer.Elapsed());

	// convert back to char (0-255) BGR
	output *= 255;
	//output.convertTo(output, CV_8UC4);
	cvtColor(output, output, CV_RGBA2BGR, 3);

	imwrite(output_file, output);
}

void processUsingCuda(std::string input_file, std::string output_file) {
	//Read input image from the disk
	cv::Mat input = cv::imread(input_file,IMREAD_UNCHANGED);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	// convert from char(0-255) BGR to float (0.0-0.1) RGBA
	Mat inputRGBA;
	cvtColor(input, inputRGBA, CV_BGR2RGBA, 4);
	inputRGBA.convertTo(inputRGBA, CV_32FC4);
	inputRGBA /= 255;

	//Create output image
	Mat output (input.size(), inputRGBA.type());

	const float euclidean_delta = 1.0f;
	const int filter_radius = 5;

	GpuTimer timer;
	timer.Start();

	bilateralFilterCuda((float4*) inputRGBA.ptr<float4>(),
			(float4*) output.ptr<float4>(),
			euclidean_delta,
			inputRGBA.cols, inputRGBA.rows,
			filter_radius);

	timer.Stop();
	printf("Own CUDA code ran in: %f msecs.\n", timer.Elapsed());

	// convert back to char (0-255) BGR
	output *= 255;
	//output.convertTo(output, CV_8UC4);
	cvtColor(output, output, CV_RGBA2BGR, 3);

	imwrite(output_file, output);
}
