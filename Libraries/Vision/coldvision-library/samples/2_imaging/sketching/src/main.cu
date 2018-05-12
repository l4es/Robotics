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
#include "opencv2/videoio.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudafilters.hpp" // cv::cuda::Filter
#include "opencv2/cudaarithm.hpp" // cv::cuda::abs or cv::cuda::addWeighted
#include "timer.h"

namespace cpu {
void processImage(std::string inputFile, std::string outputFile);
void processVideo(std::string inputFile, std::string outputFile);
}

namespace gpu {
void processImage(std::string inputFile, std::string outputFile);
void processVideo(std::string inputFile, std::string outputFile);
}

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

	for (int i=1; i<=5; ++i) {
		string fileId = std::to_string(i);
		const string inputFile = argc >= 2 ? argv[1] : "../data/img_color_"+fileId+".jpg";
		const string outputFile = argc >= 3 ? argv[2] : "../data/out_img_color_"+fileId+"_sketching.jpg";
		cpu::processImage(inputFile, outputFile);
		gpu::processImage(inputFile, outputFile);
	}

	for (int i=1; i<=5; ++i) {
		string fileId = std::to_string(i);
		const string inputFile = argc >= 2 ? argv[1] : "../data/img_ir_"+fileId+".jpg";
		const string outputFile = argc >= 3 ? argv[2] : "../data/out_img_ir_"+fileId+"_sketching.jpg";
		cpu::processImage(inputFile, outputFile);
		gpu::processImage(inputFile, outputFile);
	}

	cpu::processVideo("../data/video_color_2.mp4", "../data/out_video_color_2.mp4");
	gpu::processVideo("../data/video_color_2.mp4", "../data/out_video_color_2.mp4");

	return 0;
}

namespace cpu {

/**
 * It reduces a value to a interval value (continue to discrete value).
 * For a value between [n, n+intervalSize), where n is a multiple of intervalSize, this method returns n.
 * Example 1 (val=11, intervalSize=4): return=8
 * Example 2 (val=27, intervalSize=8): return=24
 */
inline uchar reduceValue(const uchar value, const uchar intervalSize = 4) {
	return value / intervalSize * intervalSize;
}

/**
 * It reduces the image color palette from a total number 256*256*256 of unique colors to something smaller like 32*32*32.
 * @see http://stackoverflow.com/questions/5906693/how-to-reduce-the-number-of-colors-in-an-image-with-opencv
 */
void reduceColorPalette(const Mat& inputImage, Mat& outputImage, const uchar intervalSize = 4)
{
	uchar* inputPixelPtr = inputImage.data;
	uchar* outputPixelPtr = outputImage.data;

	if(inputImage.channels() == 3) { // color image
		for (int i = 0; i < inputImage.rows; i++) {
			for (int j = 0; j < inputImage.cols; j++) {
				const int pi = i*inputImage.cols*3 + j*3;
				outputPixelPtr[pi + 0] = reduceValue(inputPixelPtr[pi + 0]); // B
				outputPixelPtr[pi + 1] = reduceValue(inputPixelPtr[pi + 1]); // G
				outputPixelPtr[pi + 2] = reduceValue(inputPixelPtr[pi + 2]); // R
			}
		}

	} else if(inputImage.channels() == 1) { // grayscale image
		for (int i = 0; i < inputImage.rows; i++) {
			for (int j = 0; j < inputImage.cols; j++) {
				const int pi = i*inputImage.cols*3 + j*3;
				outputPixelPtr[pi] = reduceValue(inputPixelPtr[pi]); // gray
			}
		}

	} else { // not supported image format
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}
}

/**
 * This method makes histogram equalization of the pixel intensities.
 * For grayscale images the equalization is done directly using the values of the grayscale channel,
 * but color images the image is transformed to other color space than RGB which separates intensity values
 * from color components, color spaces such HSV/HLS, YUV or YCbCr.
 *
 * @see http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
 */
void equalizeIntensity(const Mat& inputImage, Mat& outputImage)
{
	if(inputImage.channels() == 3) { // color image
		Mat ycrcb;
		cvtColor(inputImage, ycrcb, CV_BGR2HSV);
		vector<Mat> channels;
		split(ycrcb, channels);
		equalizeHist(channels[0], channels[0]);
		Mat result;
		merge(channels, ycrcb);
		cvtColor(ycrcb, outputImage, CV_HSV2BGR);

	} else if(inputImage.channels() == 1) { // grayscale image
		equalizeHist(inputImage, outputImage);

	} else { // not supported image format
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}
}

/**
 * This method computes the gradients of a grayscale image.
 * @param inputImage A grayscale image of type CV_8U
 * @param outputImage A grayscale image of type CV_8U
 */
void computeGradients(const Mat& inputImage, Mat& outputImage)
{
	if(inputImage.channels() == 1) { // grayscale
		// compute the gradients on both directions x and y
		Mat grad_x, grad_y;
		Mat abs_grad_x, abs_grad_y;
		int scale = 1;
		int delta = 0;
		int ddepth = CV_16S; // use 16 bits unsigned to avoid overflow

		Scharr( inputImage, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
		//Sobel( input_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_x, abs_grad_x ); // CV_16S -> CV_8U

		Scharr( inputImage, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
		//Sobel( input_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_y, abs_grad_y ); // CV_16S -> // CV_16S -> CV_8U

		// create the output by adding the absolute gradient images of each x and y direction
		addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, outputImage );

	} else {
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}

}

/**
 * This method extracts the gradient image of a color or grayscale image.
 * @param inputImage A matrix of a color or grayscale image
 * @param outputImage The gradient matrix
 */
void processImage(const Mat& inputImage, Mat& outputImage)
{
	if (inputImage.channels() == 3) { // color image
		/// Apply Histogram Equalization
		//equalizeIntensity(inputImage, inputImage);

		// Reduce the maximum number of colors from 256*256*256 to a smaller number such 32*32*32
		//reduceNumberOfColors(inputImage, inputImage);

		// Blur the input image to remove the noise
		GaussianBlur(inputImage, inputImage, Size(9, 9), 0, 0, BORDER_DEFAULT);

		// Convert it to grayscale (CV_8UC3 -> CV_8UC1)
		Mat image_gray;
		cvtColor(inputImage, image_gray, COLOR_BGR2GRAY);

		// Compute the gradient image
		computeGradients(image_gray, image_gray);
		normalize(image_gray, outputImage, 0, 255, NORM_MINMAX, CV_8U);
		//threshold(outputImage, outputImage, 50, 255, THRESH_TOZERO);

		// invert the gradient image
		cv::subtract(cv::Scalar::all(255), outputImage, outputImage);

	} else if (inputImage.channels() == 1) { // grayscale image
		Mat image_gray = inputImage;
		computeGradients(image_gray, image_gray);
		normalize(image_gray, outputImage, 0, 255, NORM_MINMAX, CV_8U);
		//threshold(outputImage, outputImage, 50, 255, THRESH_TOZERO);

		// invert the gradient image
		cv::subtract(cv::Scalar::all(255), outputImage, outputImage);

	} else { // not supported image format
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}
}

/**
 * This method extracts the gradient image of a color or grayscale image.
 * The output image is saved as grayscale.
 */
void processImage(std::string inputFile, std::string outputFile) {
	printf("CPU::Processing image: %s ...\n", inputFile.c_str());

	// Read the file
	Mat inputImage = imread(inputFile, CV_LOAD_IMAGE_UNCHANGED);
	if (!inputImage.data) {
		printf("Could not open image file: %s\n", inputFile.c_str());
		return;
	}

	// Init the output image as grayscale with the same size as the input
	Mat outputImage (inputImage.size(), CV_8U);

	// Process the image
	GpuTimer timer;
	timer.Start();
	processImage(inputImage, outputImage);
	timer.Stop();

	printf("Method processImage() ran in: %f msecs, image size: %ux%u, msecs/pixel: %f .\n",
			timer.Elapsed(), inputImage.cols, inputImage.rows, timer.Elapsed()/(inputImage.rows*inputImage.cols));

	// Display the output image
	//imshow("Final Result", outputImage);

	// Wait until the user presses a key
	//waitKey(0);

	imwrite(outputFile, outputImage);
}

/**
 * This method processes each frame of a color or grayscale video and saves them in a gradient video.
 * The output video is saved in a color format even if each frame was grayscale.
 */
void processVideo(std::string inputFile, std::string outputFile) {
	printf("CPU::Processing video: %s ...\n", inputFile.c_str());

	// Read the file
	VideoCapture inputVideo (inputFile);
	if (!inputVideo.isOpened()) {
		printf("Could not open video file: %s\n", inputFile.c_str());
		return;
	}

	// Init the output video with the same properties as the input
	int fourcc = inputVideo.get(CAP_PROP_FOURCC);
	double fps = inputVideo.get(CAP_PROP_FPS);
	Size frameSize (inputVideo.get(CAP_PROP_FRAME_WIDTH), inputVideo.get(CAP_PROP_FRAME_HEIGHT));
	VideoWriter outputVideo(outputFile, fourcc, fps, frameSize, true);
	if (!outputVideo.isOpened()) {
		printf("Could not open the output video for write: %s\n", outputFile.c_str());
		return;
	}

	// Process the video, frame by frame
	GpuTimer timer;
	timer.Start();
	Mat frame, gradientFrame;
	while (inputVideo.read(frame)) {
		gradientFrame = Mat(frame.size(),CV_8U); // as grayscale
		processImage(frame, gradientFrame); // extract grayscale gradient from the color frame
		cvtColor(gradientFrame, frame, COLOR_GRAY2BGR); // convert grayscale gradient to color
		outputVideo.write(frame);
	}
	timer.Stop();

	printf("Method processVideo() ran in: %f msecs, video size: %ux%u, total frames: %f, msecs/pixel: %f .\n",
			timer.Elapsed(), frameSize.width, frameSize.height, inputVideo.get(CAP_PROP_FRAME_COUNT),
			timer.Elapsed()/(frameSize.height*frameSize.width*inputVideo.get(CAP_PROP_FRAME_COUNT)));

	outputVideo.release();
	inputVideo.release();
}

}

namespace gpu {

/**
 * This method makes histogram equalization of the pixel intensities.
 * For grayscale images the equalization is done directly using the values of the grayscale channel,
 * but color images the image is transformed to other color space than RGB which separates intensity values
 * from color components, color spaces such HSV/HLS, YUV or YCbCr.
 *
 * @see http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
 */
void equalizeIntensity(const cv::cuda::GpuMat& inputImage, cv::cuda::GpuMat& outputImage)
{
	if(inputImage.channels() == 3) { // color image
		cv::cuda::GpuMat ycrcb;
		cv::cuda::cvtColor(inputImage, ycrcb, CV_BGR2HSV);
		vector<cv::cuda::GpuMat> channels;
		cv::cuda::split(ycrcb, channels);
		cv::cuda::equalizeHist(channels[0], channels[0]);
		cv::cuda::merge(channels, ycrcb);
		cv::cuda::cvtColor(ycrcb, outputImage, CV_HSV2BGR);
		ycrcb.release();

	} else if(inputImage.channels() == 1) { // grayscale image
		cv::cuda::equalizeHist(inputImage, outputImage);

	} else { // not supported image format
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}
}

/**
 * This method computes the gradients of a grayscale image.
 * @param inputImage A grayscale image of type CV_8U
 * @param outputImage A grayscale image of type CV_8U
 */
void computeGradients(const cv::cuda::GpuMat& inputImage, cv::cuda::GpuMat& outputImage)
{
	if(inputImage.channels() == 1) { // grayscale

		// compute the gradients on both directions x and y
		cv::cuda::GpuMat grad_x, grad_y;
		cv::cuda::GpuMat abs_grad_x, abs_grad_y;
		int scale = 1;
		int ddepth = CV_16S; // use 16 bits unsigned to avoid overflow
		Ptr<cv::cuda::Filter> filter;

		// gradient x direction
		//filter = cv::cuda::createSobelFilter(inputImage.type(), ddepth, 1, 0, 3, scale, BORDER_DEFAULT);
		filter = cv::cuda::createScharrFilter(inputImage.type(), ddepth, 1, 0, scale, BORDER_DEFAULT);
		filter->apply(inputImage, grad_x);
		cv::cuda::abs(grad_x, grad_x);
		grad_x.convertTo(abs_grad_x, CV_8UC1); // CV_16S -> CV_8U

		// gradient y direction
		//filter = cv::cuda::createSobelFilter(inputImage.type(), ddepth, 0, 1, 3, scale, BORDER_DEFAULT);
		filter = cv::cuda::createScharrFilter(inputImage.type(), ddepth, 0, 1, scale, BORDER_DEFAULT);
		filter->apply(inputImage, grad_y);
		cv::cuda::abs(grad_y, grad_y);
		grad_y.convertTo(abs_grad_y, CV_8UC1); // CV_16S -> CV_8U

		// create the output by adding the absolute gradient images of each x and y direction
		cv::cuda::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, outputImage);

		// release GPU memory
		grad_x.release();
		grad_y.release();
		abs_grad_x.release();
		abs_grad_y.release();

	} else {
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}
}

/**
 * This method extracts the inverted gradient image of a color or grayscale image.
 * @param inputImage A matrix of a color or grayscale image
 * @param outputImage The inverted gradient matrix
 */
void processImage(const cv::cuda::GpuMat& inputImage, cv::cuda::GpuMat& outputImage)
{
	if (inputImage.channels() == 3) { // color image

		// All transformations are done on the output image
		outputImage = inputImage;

		/// Apply Histogram Equalization
		//equalizeIntensity(outputImage, outputImage);

		// Reduce the maximum number of colors from 256*256*256 to a smaller number such 32*32*32
		//reduceNumberOfColors(inputImage, inputImage);

		// Blur the input image to remove the noise
		Ptr<cv::cuda::Filter> filter = cv::cuda::createGaussianFilter(outputImage.type(), outputImage.type(), Size(9,9), 0);
		filter->apply(outputImage, outputImage);

		// Convert it to grayscale (CV_8UC3 -> CV_8UC1)
		cv::cuda::cvtColor(outputImage, outputImage, COLOR_RGB2GRAY);

		// Compute the gradient image
		computeGradients(outputImage, outputImage);
		//normalize(outputImage, outputImage, 0, 255, NORM_MINMAX, CV_8U);
		//cv::cuda::threshold(outputImage, outputImage, 50, 255, THRESH_TOZERO);

		// invert the gradient image
		cv::cuda::subtract(cv::Scalar::all(255), outputImage, outputImage);

	} else if (inputImage.channels() == 1) { // grayscale image
		computeGradients(inputImage, outputImage);
		//normalize(image_gray, outputImage, 0, 255, NORM_MINMAX, CV_8U);
		//threshold(outputImage, outputImage, 50, 255, THRESH_TOZERO);

		// invert the gradient image
		cv::cuda::subtract(cv::Scalar::all(255), outputImage, outputImage);

	} else { // not supported image format
		printf("Image type not supported.\n");
		outputImage = inputImage;
	}
}

/**
 * This method extracts the gradient image of a color or grayscale image.
 * The output image is saved as grayscale.
 */
void processImage(std::string inputFile, std::string outputFile) {
	printf("GPU::Processing image: %s ...\n", inputFile.c_str());

	// Read the file
	Mat inputImage = imread(inputFile, CV_LOAD_IMAGE_UNCHANGED);
	if (!inputImage.data) {
		printf("Could not open image file: %s\n", inputFile.c_str());
		return;
	}

	// Init the output image as grayscale with the same size as the input
	Mat outputImage (inputImage.size(), CV_8U);

	// copy the input image from CPU to GPU memory
	cuda::GpuMat gpuInputImage = cuda::GpuMat(inputImage);
	cuda::GpuMat gpuOutputImage = cuda::GpuMat(outputImage);

	// Process the image
	GpuTimer timer;
	timer.Start();
	processImage(gpuInputImage, gpuOutputImage);
	timer.Stop();

	printf("Method processImage() ran in: %f msecs, image size: %ux%u, msecs/pixel: %f .\n",
			timer.Elapsed(), inputImage.cols, inputImage.rows, timer.Elapsed()/(inputImage.rows*inputImage.cols));

	// copy the result gradient from GPU to CPU and release GPU memory
	gpuOutputImage.download(outputImage);

	gpuInputImage.release();
	gpuOutputImage.release();

	// Display the output image
	//imshow("Final Result", outputImage);

	// Wait until the user presses a key
	//waitKey(0);

	imwrite(outputFile, outputImage);
}

/**
 * This method processes each frame of a color or grayscale video and saves them in a gradient video.
 * The output video is saved in a color format even if each frame was grayscale.
 */
void processVideo(std::string inputFile, std::string outputFile) {
	printf("GPU::Processing video: %s ...\n", inputFile.c_str());

	// Read the file
	VideoCapture inputVideo (inputFile);
	if (!inputVideo.isOpened()) {
		printf("Could not open video file: %s\n", inputFile.c_str());
		return;
	}

	// Init the output video with the same properties as the input
	int fourcc = inputVideo.get(CAP_PROP_FOURCC);
	double fps = inputVideo.get(CAP_PROP_FPS);
	Size frameSize (inputVideo.get(CAP_PROP_FRAME_WIDTH), inputVideo.get(CAP_PROP_FRAME_HEIGHT));
	VideoWriter outputVideo(outputFile, fourcc, fps, frameSize, true);
	if (!outputVideo.isOpened()) {
		printf("Could not open the output video for write: %s\n", outputFile.c_str());
		return;
	}

	// Process the video, frame by frame
	GpuTimer timer;
	timer.Start();
	Mat frame;
	while (inputVideo.read(frame)) {

		// copy the input image from CPU to GPU memory
		cv::cuda::GpuMat gpuFrame = cv::cuda::GpuMat(frame);
		cv::cuda::GpuMat gpuGradientFrame;

		processImage(gpuFrame, gpuGradientFrame); // extract grayscale gradient from the color frame
		cv::cuda::cvtColor(gpuGradientFrame, gpuFrame, COLOR_GRAY2BGR); // convert grayscale gradient to color

		// copy the result gradient from GPU to CPU and release GPU memory
		gpuFrame.download(frame);

		// release memory from GPU
		gpuGradientFrame.release();
		gpuFrame.release();

		// save current frame into the video
		outputVideo.write(frame);
	}
	timer.Stop();

	printf("Method processVideo() ran in: %f msecs, video size: %ux%u, total frames: %f, msecs/pixel: %f .\n",
			timer.Elapsed(), frameSize.width, frameSize.height, inputVideo.get(CAP_PROP_FRAME_COUNT),
			timer.Elapsed()/(frameSize.height*frameSize.width*inputVideo.get(CAP_PROP_FRAME_COUNT)));

	outputVideo.release();
	inputVideo.release();
}

}

