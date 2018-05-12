#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "timer.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void readme()
{
	std::cout << "Usage: ./surf_features <object_img> <scene_img>" << std::endl;
}

// It searches for the right position, orientation and scale of the object in the scene based on the good_matches.
void localizeInImage(const std::vector<DMatch>& good_matches,
		const std::vector<KeyPoint>& keypoints_object,
		const std::vector<KeyPoint>& keypoints_scene, const Mat& img_object,
		const Mat& img_matches)
{
	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	for (int i = 0; i < good_matches.size(); i++) {
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}

	try {
		Mat H = findHomography(obj, scene, RANSAC);
		//-- Get the corners from the image_1 ( the object to be "detected" )
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(img_object.cols, 0);
		obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
		obj_corners[3] = cvPoint(0, img_object.rows);
		std::vector<Point2f> scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);
		// Draw lines between the corners (the mapped object in the scene - image_2 )
		line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
				scene_corners[1] + Point2f(img_object.cols, 0),
				Scalar(255, 0, 0), 4);
		line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
				scene_corners[2] + Point2f(img_object.cols, 0),
				Scalar(255, 0, 0), 4);
		line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
				scene_corners[3] + Point2f(img_object.cols, 0),
				Scalar(255, 0, 0), 4);
		line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
				scene_corners[0] + Point2f(img_object.cols, 0),
				Scalar(255, 0, 0), 4);
	} catch (Exception& e) {}
}

/**
 * It searches for on object inside an scene using the SURF for keypoints and descriptors detection, and FLANN+KNN for matching them.
 * This implementation is done only in CPU.
 *
 * @param objectInputFile Path of the image containing the object to be searched.
 * @param sceneInputFile Path of the image where the object to be searched.
 * @param outputFile Path of the image where the matching to be saved.
 * @param minHessian The Hessian value of the SURF algorithm.
 */
void processWithCpu(string objectInputFile, string sceneInputFile, string outputFile, int minHessian = 100)
{
	printf("CPU::Processing object: %s and scene: %s ...\n", objectInputFile.c_str(), sceneInputFile.c_str());

	// Load the image from the disk
	Mat img_object = imread( objectInputFile, IMREAD_GRAYSCALE ); // surf works only with grayscale images
	Mat img_scene = imread( sceneInputFile, IMREAD_GRAYSCALE );

	if( !img_object.data || !img_scene.data ) {
		std::cout<< "Error reading images." << std::endl;
		return;
	}

	// Start the timer
	GpuTimer timer;
	timer.Start();

	vector<KeyPoint> keypoints_object, keypoints_scene; // keypoints
	Mat descriptors_object, descriptors_scene; // descriptors (features)

	//-- Steps 1 + 2, detect the keypoints and compute descriptors, both in one method
	Ptr<SURF> surf = SURF::create( minHessian );
	surf->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
	surf->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher; // FLANN - Fast Library for Approximate Nearest Neighbors
	vector< vector< DMatch> > matches;
	matcher.knnMatch( descriptors_object, descriptors_scene, matches, 2 ); // find the best 2 matches of each descriptor

	timer.Stop();
	printf( "Method processWithCpu() ran in: %f msecs, object size: %ux%u, scene size: %ux%u\n",
			timer.Elapsed(), img_object.cols, img_object.rows, img_scene.cols, img_scene.rows );

	//-- Step 4: Select only goot matches
	std::vector< DMatch > good_matches;
	for (int k = 0; k < std::min(descriptors_scene.rows - 1, (int)matches.size()); k++)
	{
		if ( (matches[k][0].distance < 0.6*(matches[k][1].distance)) &&
				((int)matches[k].size() <= 2 && (int)matches[k].size()>0) )
		{
			// take the first result only if its distance is smaller than 0.6*second_best_dist
			// that means this descriptor is ignored if the second distance is bigger or of similar
			good_matches.push_back( matches[k][0] );
		}
	}

	//-- Step 5: Draw lines between the good matching points
	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::DEFAULT );

	//-- Step 6: Localize the object inside the scene image with a square
	localizeInImage( good_matches, keypoints_object, keypoints_scene, img_object, img_matches );

	//-- Step 7: Show/save matches
	//imshow("Good Matches & Object detection", img_matches);
	//waitKey(0);
	imwrite(outputFile, img_matches);
}

/**
 * It searches for on object inside an scene using the SURF for keypoints and descriptors detection, and BruteForce+KNN for matching them.
 * This implementation is done partly in GPU (SURF+KNN) and partly in CPU (best points selection + localization).
 *
 * @param objectInputFile Path of the image containing the object to be searched.
 * @param sceneInputFile Path of the image where the object to be searched.
 * @param outputFile Path of the image where the matching to be saved.
 * @param minHessian The Hessian value of the SURF algorithm.
 */
void processWithGpu(string objectInputFile, string sceneInputFile, string outputFile, int minHessian = 100)
{
	printf("GPU::Processing object: %s and scene: %s ...\n", objectInputFile.c_str(), sceneInputFile.c_str());

	// Load the image from the disk
	Mat img_object = imread( objectInputFile, IMREAD_GRAYSCALE ); // surf works only with grayscale images
	Mat img_scene = imread( sceneInputFile, IMREAD_GRAYSCALE );
	if( !img_object.data || !img_scene.data ) {
		std::cout<< "Error reading images." << std::endl;
		return;
	}

	// Copy the image into GPU memory
	cuda::GpuMat img_object_Gpu( img_object );
	cuda::GpuMat img_scene_Gpu( img_scene );

	// Start the timer - the time moving data between GPU and CPU is added
	GpuTimer timer;
	timer.Start();

	cuda::GpuMat keypoints_scene_Gpu, keypoints_object_Gpu; // keypoints
	cuda::GpuMat descriptors_scene_Gpu, descriptors_object_Gpu; // descriptors (features)

	//-- Steps 1 + 2, detect the keypoints and compute descriptors, both in one method
	cuda::SURF_CUDA surf( minHessian );
	surf( img_object_Gpu, cuda::GpuMat(), keypoints_object_Gpu, descriptors_object_Gpu );
	surf( img_scene_Gpu, cuda::GpuMat(), keypoints_scene_Gpu, descriptors_scene_Gpu );
	//cout << "FOUND " << keypoints_object_Gpu.cols << " keypoints on object image" << endl;
	//cout << "Found " << keypoints_scene_Gpu.cols << " keypoints on scene image" << endl;

	//-- Step 3: Matching descriptor vectors using BruteForceMatcher
	Ptr< cuda::DescriptorMatcher > matcher = cuda::DescriptorMatcher::createBFMatcher();
	vector< vector< DMatch> > matches;
	matcher->knnMatch(descriptors_object_Gpu, descriptors_scene_Gpu, matches, 2);

	// Downloading results  Gpu -> Cpu
	vector< KeyPoint > keypoints_scene, keypoints_object;
	//vector< float> descriptors_scene, descriptors_object;
	surf.downloadKeypoints(keypoints_scene_Gpu, keypoints_scene);
	surf.downloadKeypoints(keypoints_object_Gpu, keypoints_object);
	//surf.downloadDescriptors(descriptors_scene_Gpu, descriptors_scene);
	//surf.downloadDescriptors(descriptors_object_Gpu, descriptors_object);

	timer.Stop();
	printf( "Method processWithGpu() ran in: %f msecs, object size: %ux%u, scene size: %ux%u\n",
			timer.Elapsed(), img_object.cols, img_object.rows, img_scene.cols, img_scene.rows );

	//-- Step 4: Select only goot matches
	//vector<Point2f> obj, scene;
	std::vector< DMatch > good_matches;
	for (int k = 0; k < std::min(keypoints_object.size()-1, matches.size()); k++)
	{
		if ( (matches[k][0].distance < 0.6*(matches[k][1].distance)) &&
				((int)matches[k].size() <= 2 && (int)matches[k].size()>0) )
		{
			// take the first result only if its distance is smaller than 0.6*second_best_dist
			// that means this descriptor is ignored if the second distance is bigger or of similar
			good_matches.push_back(matches[k][0]);
		}
	}

	//-- Step 5: Draw lines between the good matching points
	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::DEFAULT );

	//-- Step 6: Localize the object inside the scene image with a square
	localizeInImage( good_matches, keypoints_object, keypoints_scene, img_object, img_matches );

	//-- Step 7: Show/save matches
	//imshow("Good Matches & Object detection", img_matches);
	//waitKey(0);
	imwrite(outputFile, img_matches);

	//-- Step 8: Release objects from the GPU memory
	surf.releaseMemory();
	matcher.release();
	img_object_Gpu.release();
	img_scene_Gpu.release();
}

int main( int argc, char** argv )
{
	string fileId = std::to_string(3);
	string objectInputFile = "../data/img_color_"+fileId+"_object1.jpg";
	string sceneInputFile = "../data/img_color_"+fileId+".jpg";
	string outputFile = "../data/img_color_"+fileId+"_surf_features.jpg";

	processWithCpu(objectInputFile, sceneInputFile, outputFile, 100);
	processWithGpu(objectInputFile, sceneInputFile, outputFile, 100);

	return 0;
}



