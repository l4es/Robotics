/*
 * watershed_mamba.h
 *
 * This sources implement e unified version of waterfalls, standard
 * and P algorithms implemented by the autor in C++ and Python as a part
 * of the mamba-image library. as described in this web pages:
 *
 *		http://cmm.ensmp.fr/~beucher/publi/P-Algorithm_SB_BM.pdf
 * 		http://cmm.ensmp.fr/~beucher/publi/Unified_Segmentation.pdf
 * 		https://github.com/nicolasBeucher/mamba-image/
 *
 *  Created on: April 21, 2016
 *      Author: claudiu
 */

#ifndef WATERSHED_MAMBA
#define WATERSHED_MAMBA

#include "mamba/mamba.h"

using namespace std;
using namespace cv;

void copyImage(Mat &srcImage, MB_Image &dstImage) {
	dstImage.width = srcImage.cols;
	dstImage.height = srcImage.rows;
	dstImage.depth = 32; // 4 channels of 8 bits
	size_t numBytes = srcImage.rows * srcImage.step[0];
	dstImage.pixels = new PIX8[numBytes];
	memcpy ( dstImage.pixels, srcImage.data, numBytes );
}

void copyImage(MB_Image &srcImage, Mat &dstImage) {
	size_t numBytes = dstImage.rows * dstImage.step[0];
	memcpy ( srcImage.pixels, dstImage.data, numBytes );
}

void releaseImage(MB_Image &mbImage) {
	delete mbImage.pixels;
}

/**
 * This is an example method showing how to use this implementation.
 * @see http://www.mamba-image.org/examples/Moderate/Segmentation_algorithms_examples_and_demo/script.py
 * @see http://www.mamba-image.org/examples.html  (Segmentation algorithms : examples and demo)
 */
Mat watershedMamba(Mat &image) {

	if(!image.data || image.type()!= CV_8UC4) { // the type must be unsigned char with 4 channels CV_BGR2BGRA
		cout << "Invalid input image, skipping process." << endl;
		return image;
	}

	MB_Image originalImage, gradientImage, valuedWatershedImage, enhancedWatershedImage, treshholdImage;
	copyImage(image, originalImage); // this copy does initialization too, which misses in from the constructor
	copyImage(image, gradientImage);
	copyImage(image, valuedWatershedImage);
	copyImage(image, enhancedWatershedImage);
	copyImage(image, treshholdImage);

	// NOTE: Many of the algorithms are implemented in python - too much work to port it to C++ !

	// First computing the valued watershed of our image gradient
	// gradient(originalImage, gradientImage);
	// valuedWatershed(gradientImage, valuedWatershedImage);

	// Enhanced Waterfalls
	// n = enhancedWaterfalls(valuedWatershedImage, enhancedWatershedImage);
 	// threshold(enhancedWatershedImage, treshholdImage, 0, n-1);
    // print("Enhanced waterfalls, levels = %d" % (n));
    // treshholdImage.save("tools_segEW.png");

	copyImage(treshholdImage, image);

	releaseImage(originalImage);
	releaseImage(valuedWatershedImage);
	releaseImage(enhancedWatershedImage);
	releaseImage(gradientImage);
	releaseImage(treshholdImage);

	return image;
}

#endif /* WATERSHED_MAMBA */
