/*
 * main.cu
 *
 *  Created on: Nov 19, 2015
 *      Author: claudiu
 */

#include <iostream>
#include <string>
#include <stdio.h>
#include "utils.h"
#include "timer.h"
#include "grayscale.cuh"

void processUsingCuda(std::string input_file, std::string output_file);
void processUsingOpenCV(std::string input_file, std::string output_file);
void processUsingCpu(std::string input_file, std::string output_file);

int main(int argc, char **argv) {

	std::string input_file;
	std::string output_cuda_file  = "../data/output_cuda.png";
	std::string output_opencv_file = "../data/output_opencv.png";
	std::string output_cpu_file = "../data/output_cpu.png";

	// used for the allowed error between different implementations
	bool useEpsCheck = true; // set true to enable perPixelError and globalError
	double perPixelError = 3;
	double globalError   = 10;

	switch (argc)
	{
	case 2:
		input_file = std::string(argv[1]);
		break;
	case 3:
		input_file  = std::string(argv[1]);
		output_cuda_file = std::string(argv[2]);
		break;
	case 4:
		input_file  = std::string(argv[1]);
		output_cuda_file = std::string(argv[2]);
		output_opencv_file = std::string(argv[3]);
		break;
	case 5:
		input_file  = std::string(argv[1]);
		output_cuda_file = std::string(argv[2]);
		output_opencv_file = std::string(argv[3]);
		output_cpu_file = std::string(argv[4]);
		break;
	case 7:
		useEpsCheck=true;
		input_file  = std::string(argv[1]);
		output_cuda_file = std::string(argv[2]);
		output_opencv_file = std::string(argv[3]);
		output_cpu_file = std::string(argv[4]);
		perPixelError = atof(argv[5]);
		globalError   = atof(argv[6]);
		break;
	default:
		std::cerr << "Usage: ./grayscale input_file [output_cuda] [output_opencv] [output_opencv] [output_cpu] [globalError]" << std::endl;
		exit(1);
	}

	for (int i=0; i<10; ++i) {
		processUsingOpenCV(input_file, output_opencv_file);
		processUsingCuda(input_file, output_cuda_file);
		processUsingCpu(input_file, output_cpu_file);
	}

	// check if the generated images are the same
	compareImages(output_cpu_file, output_cuda_file, useEpsCheck, perPixelError, globalError);

	cleanupCuda();

	return 0;
}

void processUsingCuda(std::string input_file, std::string output_file) {
	// pointers to images in CPU's memory (h_) and GPU's memory (d_)
	uchar4        *h_rgbaImage, *d_rgbaImage;
	unsigned char *h_greyImage, *d_greyImage;

	//load the image and give us our input and output pointers
	preProcess(&h_rgbaImage, &h_greyImage, &d_rgbaImage, &d_greyImage, input_file);

	GpuTimer timer;
	timer.Start();
	// here is where the conversion actually happens
	rgbaToGreyscaleCuda(h_rgbaImage, d_rgbaImage, d_greyImage, numRows(), numCols());
	timer.Stop();
	cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());

	int err = printf("Implemented CUDA code ran in: %f msecs.\n", timer.Elapsed());

	if (err < 0) {
		//Couldn't print!
		std::cerr << "Couldn't print timing information! STDOUT Closed!" << std::endl;
		exit(1);
	}

	size_t numPixels = numRows()*numCols();
	checkCudaErrors(cudaMemcpy(h_greyImage, d_greyImage, sizeof(unsigned char) * numPixels, cudaMemcpyDeviceToHost));

	//check results and output the grey image
	postProcess(output_file, h_greyImage);
}

void processUsingOpenCV(std::string input_file, std::string output_file) {
	cv::Mat image;
	image = cv::imread(input_file.c_str(), CV_LOAD_IMAGE_COLOR);
	if (image.empty()) {
		std::cerr << "Couldn't open file: " << input_file << std::endl;
		exit(1);
	}

	GpuTimer timer;
	timer.Start();
	cv::cvtColor(image, imageRGBA, CV_BGR2RGBA);  // CV_BGR2GRAY

	//allocate memory for the output
	imageGrey.create(image.rows, image.cols, CV_8UC1);
	timer.Stop();

	int err = printf("OpenCV code ran in: %f msecs.\n", timer.Elapsed());

	//This shouldn't ever happen given the way the images are created
	//at least based upon my limited understanding of OpenCV, but better to check
	if (!imageRGBA.isContinuous() || !imageGrey.isContinuous()) {
		std::cerr << "Images aren't continuous!! Exiting." << std::endl;
		exit(1);
	}

	//output the image
	cv::imwrite(output_file.c_str(), imageGrey);
}

void processUsingCpu(std::string input_file, std::string output_file) {
	// pointers to images in CPU's memory (h_) and GPU's memory (d_)
	uchar4        *h_rgbaImage, *d_rgbaImage;
	unsigned char *h_greyImage, *d_greyImage;

	//load the image and give us our input and output pointers
	preProcess(&h_rgbaImage, &h_greyImage, &d_rgbaImage, &d_greyImage, input_file);

	GpuTimer timer;
	timer.Start();
	rgbaToGreyscaleCpu(h_rgbaImage, h_greyImage, numRows(), numCols());
	timer.Stop();

	int err = printf("Implemented CPU serial code ran in: %f msecs.\n", timer.Elapsed());

	if (err < 0) {
		//Couldn't print!
		std::cerr << "Couldn't print timing information! STDOUT Closed!" << std::endl;
		exit(1);
	}

	//check results and output the grey image
	postProcess(output_file, h_greyImage);
}
