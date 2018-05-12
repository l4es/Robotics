/*
 *  quadwindowmodule.cpp
 *  
 *
 *  Created by:
 *  Gennari Cristiano
 *  Paparo Valentina
 *  Sbandi Angelo
 *
 *  Copyright 2009 RoCoCo LAB. All rights reserved.
 *
 */
 
#define MODULE_NAME "QuadWindowModule"

#include "quadwindowmodule.h"
#include "quadwindowmodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#include <rdkcore/geometry/utils.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/time/time.h>
#include <cstring>

#define LOGGING_MODULE MODULE_NAME

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <string.h>


namespace RDK2 { namespace RAgent {
	
	IplImage* original;
	IplImage* frame;
	CvMemStorage* storage;
	CvSeq* lines;
	
	IplImage* dst;
#ifndef OpenRDK_ARCH_ARM9
	IplImage* color_dst;
#endif

	double rho;
	double theta;
	int thresh;
	double p1;
	double p2;
	
	int vert;
	int oriz;
	int width;
	int height;
	int obl;
	
	string prefix;
	int counter = 0;
	bool targetFound = false;
	
bool QuadWindowModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, false);
		// here you declare the properties of your module
		// in the repository they have the name of this module as prefix, e.g. /yourModuleName/maxSpeed
		
		//PARAMS
		session->createInt(PARAM_WIDTH, "Image WIDTH", 320);
		session->createInt(PARAM_HEIGHT, "Image HEIGHT", 240);
		
		session->createInt(PARAM_THRESHOLD_THRESH, "Threshold", 135);
		
		session->createInt(PARAM_ED_DILATE, "Dilate", 3);
		session->createInt(PARAM_ED_ERODE, "Erode", 2);
		
		session->createDouble(PARAM_HOUGH_RHO, "Hough rho", RDouble::OTHER, 1);
		session->createDouble(PARAM_HOUGH_THETA, "Hough theta", RDouble::OTHER, 210);
		session->createInt(PARAM_HOUGH_THRESHOLD, "Hough threshold", 30);
		session->createDouble(PARAM_HOUGH_P1, "Hough param 1", RDouble::OTHER, 1);
		session->createDouble(PARAM_HOUGH_P2, "Hough param 2", RDouble::OTHER, 70);
		
		session->createInt(PARAM_GAUSS_WIDTH, "Gaussian width", 9);
		session->createInt(PARAM_GAUSS_HEIGHT, "Gaussian height", 9);
		
		session->createInt(PARAM_RANGE_X, "Left and right margin", 50);
		session->createInt(PARAM_RANGE_Y, "Up and down margin", 50);
		session->createInt(PARAM_RANGE_OBL, "Oblique line threshold", 32);
		
		session->createString(PARAM_FILE_PREFIX, "Filename of photo prefix", "photosQWIN/QWIN");
		
		//PROPERTIES
		session->createImage(PROPERTY_IMAGE_IN, "Image to process", session->getInt(PARAM_WIDTH), session->getInt(PARAM_HEIGHT), RDK2::RGraphics::RImage::RGB24);
		session->setVolatile(PROPERTY_IMAGE_IN);
		
		session->createImage(PROPERTY_IMAGE_OUT, "Final image", session->getInt(PARAM_WIDTH), session->getInt(PARAM_HEIGHT), RDK2::RGraphics::RImage::RGB24);
		session->setVolatile(PROPERTY_IMAGE_OUT);
		
		
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

// bool QuadWindowModule::initInterfaceProperties() { }

bool QuadWindowModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		// here you can declare the events you are waiting in the main exec() loop,
		// for example:
		session->lock(PROPERTY_IMAGE_IN, HERE);
		RImage *image = session->getObjectAsL<RImage>(PROPERTY_IMAGE_IN);
		image->setSizeAndType(session->getInt(PARAM_WIDTH), session->getInt(PARAM_HEIGHT), image->getType());
		session->unlock(PROPERTY_IMAGE_IN);
		
		session->lock(PROPERTY_IMAGE_OUT, HERE);
		image = session->getObjectAsL<RImage>(PROPERTY_IMAGE_OUT);
		image->setSizeAndType(session->getInt(PARAM_WIDTH), session->getInt(PARAM_HEIGHT), image->getType());
		session->unlock(PROPERTY_IMAGE_OUT);
		
		original= cvCreateImage(cvSize(session->getInt(PARAM_WIDTH), session->getInt(PARAM_HEIGHT)), 8, 3);
		storage = cvCreateMemStorage(0);
		lines = 0;
		
		frame = cvCreateImage(cvGetSize(original), 8, 3 );
		dst = cvCreateImage(cvGetSize(original), 8, 1 );

#ifndef OpenRDK_ARCH_ARM9
		color_dst = cvCreateImage(cvGetSize(original), 8, 3 );
#endif

		//Main loop
		session->listen(PROPERTY_IMAGE_IN);
		
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void QuadWindowModule::exec()
{
	TimerT timer;
	SESSION_TRY_START(session)
		while (session->wait(), !exiting) {
			if (session->getBool(PROPERTY_MODULE_ENABLED)) {
				timer.start();
				session->lock(PROPERTY_IMAGE_IN, HERE);
				RImage *imageIn = session->getObjectAsL<RImage>(PROPERTY_IMAGE_IN);
				unsigned char *bufIn = imageIn->getBuffer();
				memcpy(original->imageData, bufIn, original->imageSize);
				session->unlock(PROPERTY_IMAGE_IN);
				
				cvCvtColor(original, frame, CV_RGB2HSV);
				cvSplit(frame, dst, NULL, NULL, NULL);
				cvSmooth(dst,dst,CV_GAUSSIAN,session->getInt(PARAM_GAUSS_WIDTH), session->getInt(PARAM_GAUSS_HEIGHT));
				cvThreshold(dst, dst, session->getInt(PARAM_THRESHOLD_THRESH), 255, CV_THRESH_BINARY);

				cvCanny(dst, dst, 50, 200, 3 );
				
#ifndef OpenRDK_ARCH_ARM9
				cvCvtColor(dst, color_dst, CV_GRAY2BGR );
#endif

				rho = session->getDouble(PARAM_HOUGH_RHO);
				theta = session->getDouble(PARAM_HOUGH_THETA);
				thresh = session->getInt(PARAM_HOUGH_THRESHOLD);
				p1 = session->getDouble(PARAM_HOUGH_P1);
				p2 = session->getDouble(PARAM_HOUGH_P2);
				lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, rho, CV_PI/theta, thresh, p1, p2 );
				
				vert = 0;
				oriz = 0;
				width = session->getInt(PARAM_WIDTH);
				height = session->getInt(PARAM_HEIGHT);
				obl = session->getInt(PARAM_RANGE_OBL);
				for(int i = 0; i < lines->total; i++ ) {
					CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
					if(abs(((line[0]).x)-((line[1]).x)) < obl) {  //linee verticali
						if((((((line[0]).x) + ((line[1]).x))/2) < session->getInt(PARAM_RANGE_X)) ||
							(((((line[0]).x) + ((line[1]).x))/2) > (width - session->getInt(PARAM_RANGE_X)) )){
#ifndef OpenRDK_ARCH_ARM9
							cvLine( color_dst, line[0], line[1], CV_RGB(255,0,255), 3, CV_AA, 0 );
#endif
							vert++;
						}
					}
					else
						if(abs(((line[0]).y)-((line[1]).y)) < obl/4*3) { //linee orizzontali
							if((((((line[0]).y) + ((line[1]).y))/2) < session->getInt(PARAM_RANGE_Y))||
								(((((line[0]).y) + ((line[1]).y))/2) > (height - session->getInt(PARAM_RANGE_Y)))){
#ifndef OpenRDK_ARCH_ARM9
								cvLine(color_dst, line[0], line[1], CV_RGB(255,255,0), 3, CV_AA, 0);
#endif
								oriz++;
							}
						}//endif
					
				}//end for
				
				CvPoint center;
				center.x=width/2;
				center.y=height/2;
				if((vert>1)&&(oriz>1)) {
#ifndef OpenRDK_ARCH_ARM9
					cvCircle(color_dst,center,60,CV_RGB(0,255,0),CV_FILLED,4);
#endif					
					RDK_TRACE_STREAM("Target found");
					ostringstream off;
					int t=floor(Timestamp().getMsFromMidnight());
					int sec=floor(t/1000);
					int min = floor(sec/60);
					int h=floor(min/60);
					prefix = session->getString(PARAM_FILE_PREFIX);
					off << prefix << h << "-" << min % 60 << "-" << sec % 60 << "_" << t % 1000 << "(" << fixed << counter <<").jpg";
					cvCvtColor(original, original, CV_BGR2RGB);
					cvSaveImage(off.str().c_str(), original);
					counter++;
				}
				
#ifndef OpenRDK_ARCH_ARM9
				/////REPOSITORY FINAL IMAGE
				session->lock(PROPERTY_IMAGE_OUT, HERE);
				
				cvCvtColor(color_dst, color_dst, CV_BGR2RGB);
				RImage *imageOut = session->getObjectAsL<RImage>(PROPERTY_IMAGE_OUT);
				unsigned char *bufOut = imageOut->getBuffer();
				memcpy(bufOut, color_dst->imageData, color_dst->imageSize);
				
				session->unlock(PROPERTY_IMAGE_OUT);
				session->valueChanged(PROPERTY_IMAGE_OUT);
#endif
				////FINE REPOSITORY
			}//end if enabled
			
			//session->listen(PROPERTY_IMAGE_IN);
			
			RDK_TRACE_STREAM(MODULE_NAME << " exec time:" << timer.getMs() << "ms");
			
		}//while (session->wait(), !exiting)
	SESSION_END_CATCH_TERMINATE(session)
}//exec method

// implement this if you need to force the exec to go out of the loop
// if the thread may be waiting not for the session->wait() semaphore:
// on closing, the main thread will set the exiting variables, call this exitRequested()
// function and then signal the session semaphores
// void QuadWindowModule::exitRequested() { }

// implement this if you need to clean up things after the exec has exited
// void QuadWindowModule::cleanup() { }

// void QuadWindowModule::asyncAgentCmd(cstr cmd)
// {
//	SESSION_TRY_START(asyncSession)
//	// here you can parse 'cmd'
//	SESSION_END_CATCH_TERMINATE(asyncSession)
// }


MODULE_FACTORY(QuadWindowModule);

}} // namespace
