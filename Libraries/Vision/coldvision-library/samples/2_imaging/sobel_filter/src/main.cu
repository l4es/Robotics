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
#include "opencv2/cudafilters.hpp" // cv::cuda::Filter
#include "opencv2/cudaarithm.hpp" // cv::cuda::abs or cv::cuda::addWeighted
#include "timer.h"
#include "sobel_filter.cuh"

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

void processUsingOpenCvCpu(std::string input_file, std::string output_file) {
	//Read input image from the disk
	Mat input = imread(input_file, CV_LOAD_IMAGE_COLOR);
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	GpuTimer timer;
	timer.Start();

	// blur the input image to remove the noise
	GaussianBlur( input, input, Size(3,3), 0, 0, BORDER_DEFAULT );

	// convert it to grayscale (CV_8UC3 -> CV_8UC1)
	Mat input_gray;
	cvtColor( input, input_gray, COLOR_RGB2GRAY );

	// compute the gradients on both directions x and y
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S; // use 16 bits unsigned to avoid overflow

	//Scharr( input_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	Sobel( input_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x ); // CV_16S -> CV_8U

	//Scharr( input_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	Sobel( input_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y ); // CV_16S -> // CV_16S -> CV_8U

	// create the output by adding the absolute gradient images of each x and y direction
	Mat output;
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, output );

	timer.Stop();
	printf("OpenCV CPU code ran in: %f msecs.\n", timer.Elapsed());

	// show image
	//imshow("Image", output);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, output);
}

void processUsingOpenCvGpu(std::string input_file, std::string output_file) {
	//Read input image from the disk
	Mat input = imread(input_file, CV_LOAD_IMAGE_COLOR);
	Mat output;
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	GpuTimer timer;
	timer.Start();

	// copy the input image from CPU to GPU memory
	cuda::GpuMat gpuInput = cuda::GpuMat(input);

	// blur the input image to remove the noise
	Ptr<cv::cuda::Filter> filter = cv::cuda::createGaussianFilter(gpuInput.type(), gpuInput.type(), Size(3,3), 0);
	filter->apply(gpuInput, gpuInput);

	// convert it to grayscale (CV_8UC3 -> CV_8UC1)
	cv::cuda::GpuMat gpuInput_gray;
	cv::cuda::cvtColor( gpuInput, gpuInput_gray, COLOR_RGB2GRAY );

	// compute the gradients on both directions x and y
	cv::cuda::GpuMat gpuGrad_x, gpuGrad_y;
	cv::cuda::GpuMat abs_gpuGrad_x, abs_gpuGrad_y;
	int scale = 1;
	int ddepth = CV_16S; // use 16 bits unsigned to avoid overflow

	// gradient x direction
	filter = cv::cuda::createSobelFilter(gpuInput_gray.type(), ddepth, 1, 0, 3, scale, BORDER_DEFAULT);
	filter->apply(gpuInput_gray, gpuGrad_x);
	cv::cuda::abs(gpuGrad_x, gpuGrad_x);
	gpuGrad_x.convertTo(abs_gpuGrad_x, CV_8UC1); // CV_16S -> CV_8U

	// gradient y direction
	filter = cv::cuda::createSobelFilter(gpuInput_gray.type(), ddepth, 0, 1, 3, scale, BORDER_DEFAULT);
	filter->apply(gpuInput_gray, gpuGrad_y);
	cv::cuda::abs(gpuGrad_y, gpuGrad_y);
	gpuGrad_y.convertTo(abs_gpuGrad_y, CV_8UC1); // CV_16S -> CV_8U

	// create the output by adding the absolute gradient images of each x and y direction
	cv::cuda::GpuMat gpuOutput;
	cv::cuda::addWeighted( abs_gpuGrad_x, 0.5, abs_gpuGrad_y, 0.5, 0, gpuOutput );

	// copy the result gradient from GPU to CPU and release GPU memory
	gpuOutput.download(output);
	gpuOutput.release();
	gpuInput.release();
	gpuInput_gray.release();
	gpuGrad_x.release();
	gpuGrad_y.release();
	abs_gpuGrad_x.release();
	abs_gpuGrad_y.release();

	timer.Stop();
	printf("OpenCV GPU code ran in: %f msecs.\n", timer.Elapsed());

	// show image
	//imshow("Image", output);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, output);
}

void processUsingCuda(std::string input_file, std::string output_file) {
	//Read input image from the disk
	Mat input = cv::imread(input_file,CV_LOAD_IMAGE_UNCHANGED);
	Mat output (input.rows, input.cols, CV_8UC1); // same size as the input but with only one channel
	if(input.empty())
	{
		std::cout<<"Image Not Found: "<< input_file << std::endl;
		return;
	}

	GpuTimer timer;
	timer.Start();

	// this method does all operations: upload to GPU, convert to grayscale, gaussian filter and sobel operator, download from GPU
	sobelFilterCuda(input, output);

	timer.Stop();
	printf("Own CUDA code ran in: %f msecs.\n", timer.Elapsed());

	//imshow("Image", output);

	// wait until user press a key
	//waitKey(0);

	imwrite(output_file, output);
}
