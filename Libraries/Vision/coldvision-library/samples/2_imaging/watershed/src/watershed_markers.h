/*
 * watershed_marker.h
 *
 * This sources implement the image segmentation using a marker-controlled Watershed algorithm
 * with an over-segmentation reduction, using OpenCV 3.X and C++ as described in this web page:
 * 		http://docs.opencv.org/3.1.0/d2/dbd/tutorial_distance_transform.html#gsc.tab=0

 *  Created on: April 20, 2016
 *      Author: claudiu
 */

#ifndef WATERSHED_MARKERS
#define WATERSHED_MARKERS

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "watershed_utils.h"

using namespace std;
using namespace cv;

/**
 * This is an example method showing how to use this implementation.
 *
 * @param input The original image.
 * @return wshedWithImage A merged image of the original and the segments.
 */
Mat watershedWithMarkers(Mat input) {

	// Change the background from white to black, since that will help later to extract
	// better results during the use of Distance Transform
	for( int x = 0; x < input.rows; x++ ) {
		for( int y = 0; y < input.cols; y++ ) {
			if ( input.at<Vec3b>(x, y) == Vec3b(255,255,255) ) {
				input.at<Vec3b>(x, y)[0] = 0;
				input.at<Vec3b>(x, y)[1] = 0;
				input.at<Vec3b>(x, y)[2] = 0;
			}
		}
	}
	// Show output image
	//imshow("Black Background Image", input);

	// Create a kernel that we will use for accuting/sharpening our image
	Mat kernel = (Mat_<float>(3,3) <<
			1,  1, 1,
			1, -8, 1,
			1,  1, 1); // an approximation of second derivative, a quite strong kernel

	// do the laplacian filtering as it is
	// well, we need to convert everything in something more deeper then CV_8U
	// because the kernel has some negative values,
	// and we can expect in general to have a Laplacian image with negative values
	// BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
	// so the possible negative number will be truncated
	Mat imgLaplacian;
	Mat sharp = input; // copy source image to another temporary one
	filter2D(sharp, imgLaplacian, CV_32F, kernel);
	input.convertTo(sharp, CV_32F);
	Mat imgResult = sharp - imgLaplacian;
	// convert back to 8bits gray scale
	imgResult.convertTo(imgResult, CV_8UC3);
	imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
	// imshow( "Laplace Filtered Image", imgLaplacian );
	//imshow( "New Sharped Image", imgResult );

	input = imgResult; // copy back
	// Create binary image from source image
	Mat bw;
	cvtColor(input, bw, CV_BGR2GRAY);
	threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//imshow("Binary Image", bw);

	// Perform the distance transform algorithm
	Mat dist;
	distanceTransform(bw, dist, CV_DIST_L2, 3);
	// Normalize the distance image for range = {0.0, 1.0}
	// so we can visualize and threshold it
	normalize(dist, dist, 0, 1., NORM_MINMAX);
	//imshow("Distance Transform Image", dist);

	// Threshold to obtain the peaks
	// This will be the markers for the foreground objects
	threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
	// Dilate a bit the dist image
	Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
	dilate(dist, dist, kernel1);
	//imshow("Peaks", dist);

	// Create the CV_8U version of the distance image
	// It is needed for findContours()
	Mat dist_8u;
	dist.convertTo(dist_8u, CV_8U);

	// Find total markers
	vector<vector<Point> > contours;
	findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Create the marker image for the watershed algorithm
	Mat markers = Mat::zeros(dist.size(), CV_32SC1);

	// Draw the foreground markers
	for (size_t i = 0; i < contours.size(); i++)
		drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);

	// Draw the background marker
	circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
	//imshow("Markers", markers*10000);

	// Perform the watershed algorithm
	watershed(input, markers);
	Mat mark = Mat::zeros(markers.size(), CV_8UC1);
	markers.convertTo(mark, CV_8UC1);
	bitwise_not(mark, mark);
	//imshow("Markers_v2", mark); // uncomment this if you want to see how the mark image looks like at that point

	int numOfSegments = contours.size();
	Mat wshed = createSegmentationDisplay(markers, numOfSegments, input);

	return wshed;
}

#endif /* WATERSHED_MARKERS */
