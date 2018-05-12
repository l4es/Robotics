/*
 * watershed_power.h
 *
 * This sources implement the image segmentation using graph cuts, random walker
 * and optimal spanning forest as described in this web pages:
 *
 * 		http://ieeexplore.ieee.org/xpl/articleDetails.jsp?tp=&arnumber=5639015
 * 		https://sourceforge.net/projects/powerwatershed/
 *
 *  Created on: April 21, 2016
 *      Author: claudiu
 */

#ifndef WATERSHED_POWER
#define WATERSHED_POWER

#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "cs_lib.h"
#include "mccodimage.h"
#include "mcimage.h"
#include "cccodimage.h"
#include "lMSF.h"
#include "MSF_RW.h"
#include "powerwatsegm.h"
#include "image_toolbox.h"
#include "unistd.h"
#include "argv.h"

using namespace std;
using namespace cv;

/**
 * This is an example method showing how to use this implementation.
 * @see src/powerwatsegm.c
 */
void watershedPower(string input_file, string output_file,
		string seeds_file,
		int algo = 2,
		bool mult = true,
		bool geod = false) {

	char * image_name = (char*) malloc(input_file.length()*sizeof(char));
	strcpy(image_name, input_file.c_str());

	char * overlay_name = (char*) malloc(output_file.length()*sizeof(char));
	strcpy(overlay_name, output_file.c_str());

	char * seeds_name = (char*) malloc(seeds_file.length()*sizeof(char));
	strcpy(seeds_name, seeds_file.c_str());

	int t1=clock();
	bool quicksort = false;
	int32_t nblabels, i,j;
	struct xvimage * image_r;
	struct xvimage * image_v;
	struct xvimage * image_b;

	struct xvimage * output = NULL;
	struct xvimage * seeds;

	unsigned char * s;
	int rs, cs, ds, N, M;

	// Reading the seed image
	seeds = readimage(seeds_name);
	if (seeds == NULL) { fprintf(stderr, "msf_rw: readimage failed\n"); exit(1); }
	s = UCHARDATA(seeds);

	rs = rowsize(seeds);
	cs = colsize(seeds);
	ds = colsize(seeds);
	bool color=false;

	int size = strlen(image_name);
	ds=1; //default
	if (strcmp(&image_name[size-3], "pgm")==0) // grey levels image
	{
		image_r = readimage(image_name);
		ds = depth(image_r);
	}
	else
	{
		color = true;
		readrgbimage(image_name,  &image_r, &image_v, &image_b);
	}
	N = rs * cs * ds;
	M = ds*rs*(cs-1)+ds*(rs-1)*cs+(ds-1)*cs*rs;  /*number of edges*/
	int ** edges;

	int * index_seeds = (int*)malloc(N*sizeof(int));
	uint8_t * index_labels = (uint8_t*)malloc(N*sizeof(uint8_t));
	j=0;
	//multilabel seed image
	if (mult == true)
	{
		nblabels = 0;
		for (i=0;i<rs*cs*ds;i++)
			if(s[i]>0)
			{
				index_seeds[j]=i;
				index_labels[j]=s[i];
				j++;
				if(s[i]>nblabels) nblabels = s[i];
			}
	}
	else
	{
		nblabels=2;
		for (i=0;i<rs*cs*ds;i++)
		{
			if(s[i]>155)
			{
				index_seeds[j] = i;
				index_labels[j]=1;
				j++;
			}
			else if(s[i]<100)
			{
				index_seeds[j] = i;
				index_labels[j]=2;
				j++;
			}
		}
	}
	int size_seeds = j;
	freeimage(seeds);
	edges =  (int**)malloc(2*sizeof(int*));
	for(i=0;i<2;i++) edges[i] = (int*)malloc(M*sizeof(int));

	compute_edges(edges,rs, cs, ds);
	if (algo == 1) // Kruskal
	{
		uint32_t * weights = (uint32_t *)malloc(sizeof(uint32_t)*M);
		int max_weight = 255;
		if (color == true)
			max_weight = color_standard_weights( image_name , weights, edges, index_seeds, size_seeds, geod, quicksort);
		else
			grey_weights(image_name, weights, edges,index_seeds, size_seeds, geod, quicksort);

		output = MSF_Kruskal(edges,weights, max_weight, index_seeds, index_labels, size_seeds, rs, cs, ds, nblabels);
		free(weights);
	}

	else if (algo == 3) // Prim RB tree
	{
		uint32_t * weights = (uint32_t *)malloc(sizeof(uint32_t)*M);
		if (color == true)
			color_standard_weights( image_name , weights, edges, index_seeds, size_seeds, geod, quicksort);
		else
			grey_weights(image_name, weights, edges,index_seeds, size_seeds, geod, quicksort);
		output = MSF_Prim(edges,weights, index_seeds, index_labels, size_seeds,rs, cs, ds, nblabels);
		free(weights);
	}

	else if (algo == 2) // Kruskal & RW on plateaus multiseeds linear time
	{
		struct xvimage * img_proba;
		uint32_t * weights = (uint32_t *)malloc(sizeof(uint32_t)*M);
		uint32_t * normal_weights ;
		uint32_t max_weight = 255;
		if (color == true) {
			normal_weights = color_standard_weights_PW( image_name , weights, edges, index_seeds, size_seeds, &max_weight, quicksort);
		} else  {
			normal_weights = grey_weights_PW(image_name,  edges,index_seeds, size_seeds, weights, quicksort);
		}

#ifndef SPEEDY

		img_proba = allocimage(NULL, rs, cs, ds, VFF_TYP_1_BYTE);
#endif
		if (geod ==true)
			output = PowerWatershed_q2(edges, weights, weights, max_weight,index_seeds, index_labels, size_seeds,rs, cs, ds, nblabels, quicksort, img_proba);
		else
			output = PowerWatershed_q2(edges, weights, normal_weights,max_weight,index_seeds, index_labels, size_seeds,rs, cs, ds, nblabels, quicksort, img_proba);
#ifndef SPEEDY
		//writeimage(img_proba, (char*)"proba.pgm");
		freeimage(img_proba);
#endif
		free(weights);
		free(normal_weights);

	}
	int t2=clock();
	assert(output != NULL);

	free(index_seeds);
	free(index_labels);
	// if (output_name == NULL)
	//    output_name =(char*)"mask.pgm";
	//writeimage(output, output_name);

	// overlay for 2D images only
	if (ds==1){
		if (overlay_name == NULL)
			overlay_name =(char*)"overlay.ppm";
		overlay(algo, image_r, image_v, image_b, output, color, overlay_name);
	}
	if (color)
	{
		freeimage(image_v);
		freeimage(image_b);
	}
	freeimage(image_r);
	freeimage(output);

	//printf("Computation time : %.6lf seconds elapsed\n", ((double)t2-t1)/CLOCKS_PER_SEC);
}

#endif /* WATERSHED_MARKERS */
