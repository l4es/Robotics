/*
 * watershed_utils.h
 *
 *  Created on: Apr 20, 2016
 *      Author: claudiu
 */

#ifndef WATERSHED_UTILS_H_
#define WATERSHED_UTILS_H_

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

Mat createSegmentationDisplay(const Mat & segments, int numOfSegments, const Mat & image)
{
	//create a new image
	Mat wshed(image.size(), CV_8UC3);

	//Create color tab for coloring the segments
	vector<Vec3b> colorTab;
	for(int i = 0; i < numOfSegments; i++ )
	{
		int b = theRNG().uniform(0, 255);
		int g = theRNG().uniform(0, 255);
		int r = theRNG().uniform(0, 255);
		colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}

	//assign different color to different segments
	for(int i = 0; i < segments.rows; i++ )
	{
		for(int j = 0; j < segments.cols; j++ )
		{
			int index = segments.at<int>(i,j);

			if( index == -1 )
				wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
			else if( index <= 0 || index > numOfSegments )
				wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
			else
				wshed.at<Vec3b>(i,j) = colorTab[index - 1];
		}
	}
	Mat normWshed;
	if (image.channels()==1) {
		cvtColor(wshed, wshed, CV_BGR2GRAY);
	}
	normalize(wshed, normWshed, 0, 255, NORM_MINMAX, image.type());

	//cout << "M = "<< endl << " "  << normWshed << endl << endl;
	//cout << "M = "<< endl << " "  << image.type() << endl << endl;
	//cout << "M = "<< endl << " "  << normWshed.type() << endl << endl;

	addWeighted(normWshed, 0.3, image, 0.7, 0, normWshed);
	return normWshed;
}


#endif /* WATERSHED_UTILS_H_ */
