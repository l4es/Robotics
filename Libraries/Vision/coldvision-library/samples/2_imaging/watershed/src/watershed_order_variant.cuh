/*
 * watershed_order_variant.cuh
 *
 * This sources implement the image segmentation using the order-variant watershed algorithm as described at this web pages:
 *
 * 		https://github.com/louismullie/watershed-cuda
 * 		http://www.fem.unicamp.br/~labaki/Academic/cilamce2009/1820-1136-1-RV.pdf
 * 		http://www.lbd.dcc.ufmg.br/colecoes/wvc/2009/0012.pdf
 *
 *  Created on: April 26, 2016
 *      Author: claudiu
 */

#ifndef WATERSHED_ORDER_VARIANT
#define WATERSHED_ORDER_VARIANT

#include "watershed_utils.h"

using namespace std;
using namespace cv;

#define INF 9999999999
#define PLATEAU 0
#define BLOCK_SIZE 6

// Texture reference for 2D float texture
texture<float, 2, cudaReadModeElementType> imageTexture;

static inline void _safe_cuda_call(cudaError err, const char* msg, const char* file_name, const int line_number)
{
	if(err!=cudaSuccess)
	{
		fprintf(stderr,"%s\n\nFile: %s\n\nLine Number: %d\n\nReason: %s\n",msg,file_name,line_number,cudaGetErrorString(err));
		std::cin.get();
		exit(EXIT_FAILURE);
	}
}

#define SAFE_CALL(call,msg) _safe_cuda_call((call),(msg),__FILE__,__LINE__)

// Convert 2D index to 1D index.
#define INDEX(j,i,ld) ((j) * ld + (i))

// Convert local (shared memory) coord to global (image) coordinate.
#define L2I(ind,off) (((ind) / BLOCK_SIZE) * (BLOCK_SIZE - 2)-1+(off))

// Texture memory for image.
texture<float, cudaTextureType2D, cudaReadModeElementType> inputTexture;

// Neighbour pixel generator (N-W to W order).
__constant__ int N_xs[8] = {-1,0,1,1,1,0,-1,-1};
__constant__ int N_ys[8] = {-1,-1,-1,0,1,1,1,0};

// Step 1.
__global__ void descentKernel(float* labeled, const size_t  w, const size_t  h)
{
	int tx = threadIdx.x;  int ty = threadIdx.y;
	int bx = blockIdx.x;   int by = blockIdx.y;
	int bdx = blockDim.x;  int bdy = blockDim.y;
	int i = bdx * bx + tx; int j = bdy * by + ty;

	__shared__ float s_I[BLOCK_SIZE*BLOCK_SIZE];
	int size = BLOCK_SIZE - 2;
	int img_x = L2I(i,tx);
	int img_y = L2I(j,ty);
	int new_w = w + w * 2;
	int new_h = h + h * 2;
	int p = INDEX(img_y,img_x,w);

	int ghost = (tx == 0 || ty == 0 ||
			tx == bdx - 1 || ty == bdy - 1);

	if ((bx == 0 && tx == 0) || (by == 0 && ty == 0) ||
			(bx == (w / size - 1) && tx == bdx - 1) ||
			(by == (h / size - 1) && ty == bdy - 1)) {
		s_I[INDEX(ty,tx,BLOCK_SIZE)] = INF;
	} else {
		s_I[INDEX(ty,tx,BLOCK_SIZE)] = tex2D(imageTexture,img_x,img_y);
	}

	__syncthreads();

	if (j < new_h && i < new_w && ghost == 0) {
		float I_q_min = INF;
		float I_p = tex2D(imageTexture,img_x,img_y);

		// find the minimum value from the neighbours of the current pixel
		int exists_q = 0;
		for (int k = 0; k < 8; k++) {
			int n_x = N_xs[k]+tx; int n_y = N_ys[k]+ty;
			float I_q = s_I[INDEX(n_y,n_x,BLOCK_SIZE)];
			if (I_q < I_q_min) I_q_min = I_q;
		}

		// set current labeled[p] to the index of the minimum
		for (int k = 0; k < 8; k++) {
			int x = N_xs[k]; int y = N_ys[k];
			int n_x = x+tx; int n_y = y+ty;
			int n_tx = L2I(i,n_x); int n_ty = L2I(j,n_y);
			float I_q = s_I[INDEX(n_y,n_x,BLOCK_SIZE)];
			int q = INDEX(n_ty,n_tx,w);
			if (I_q < I_p && I_q == I_q_min) {
				labeled[p] = -q; // store the index of the minimum point as negative
				exists_q = 1; break;
			}
		}
		// if this current pixel is a local minimum then it is marked as platueau
		if (exists_q == 0) labeled[p] = PLATEAU;
	}
}

// Step 2A.
__global__ void incrementKernel(float* L, const int w, const int h)
{
	int i = blockDim.x * blockIdx.x + threadIdx.x;
	int j = blockDim.y * blockIdx.y + threadIdx.y;
	int p = INDEX(j,i,w);

	if (j < h && i < w && L[p] == PLATEAU) {
		L[p] = p + 1;
	}
}

// Step 2B.
__global__ void minimaKernel(float* L, int* C, const int w, const int h)
{
	int tx = threadIdx.x;  int ty = threadIdx.y;
	int bx = blockIdx.x;   int by = blockIdx.y;
	int bdx = blockDim.x;  int bdy = blockDim.y;
	int i = bdx * bx + tx; int j = bdy * by + ty;

	__shared__ float s_L[BLOCK_SIZE*BLOCK_SIZE];
	int size = BLOCK_SIZE - 2;
	int img_x = L2I(i,tx);
	int img_y = L2I(j,ty);
	int true_p = INDEX(img_y,img_x,w);
	int s_p = INDEX(ty,tx,BLOCK_SIZE);
	int new_w = w + w * 2;
	int new_h = h + h * 2;
	int ghost =  (tx == 0 || ty == 0 ||
			tx == bdx - 1 || ty == bdy - 1) ? 1 : 0;

	if ((bx == 0 && tx == 0) || (by == 0 && ty == 0) ||
			(bx == (w / size - 1) && tx == bdx - 1) ||
			(by == (h / size - 1) && ty == bdy - 1)) {
		s_L[INDEX(ty,tx,BLOCK_SIZE)] = INF;
	} else {
		s_L[s_p] = L[INDEX(img_y,img_x,w)];
	}

	__syncthreads();

	int active = (j < new_h && i <
			new_w && s_L[s_p] > 0) ? 1 : 0;

	if (active == 1 && ghost == 0) {
		for (int k = 0; k < 8; k++) {
			int n_x = N_xs[k] + tx; int n_y = N_ys[k] + ty;
			int s_q = INDEX(n_y,n_x,BLOCK_SIZE);
			if (s_L[s_q] == INF) continue;
			if (s_L[s_q] > s_L[s_p])
				s_L[s_p] = s_L[s_q];
		}
		if (L[true_p] != s_L[s_p]) {
			L[true_p] = s_L[s_p];
			atomicAdd(&C[0],1);
		}
	}
}

// Step 3.
__global__ void plateauKernel(float* L, int* C, const int w, const int h)
{
	int tx = threadIdx.x;  int ty = threadIdx.y;
	int bx = blockIdx.x;   int by = blockIdx.y;
	int bdx = blockDim.x;  int bdy = blockDim.y;
	int i = bdx * bx + tx; int j = bdy * by + ty;

	__shared__ float s_L[BLOCK_SIZE*BLOCK_SIZE];
	int size = BLOCK_SIZE - 2;
	int img_x = L2I(i,tx);
	int img_y = L2I(j,ty);
	int true_p = INDEX(img_y,img_x,w);
	int p = INDEX(ty,tx,BLOCK_SIZE);
	int new_w = w + w * 2;
	int new_h = h + h * 2;
	int ghost = (tx == 0 || ty == 0 ||
			tx == bdx - 1 || ty == bdy - 1);

	// Load data into shared memory.
	if ((bx == 0 && tx == 0) || (by == 0 && ty == 0) ||
			(bx == (w / size - 1) && tx == bdx - 1) ||
			(by == (h / size - 1) && ty == bdy - 1)) {
		s_L[INDEX(ty,tx,BLOCK_SIZE)] = INF;
	} else {
		s_L[INDEX(ty,tx,BLOCK_SIZE)] =
				L[INDEX(img_y,img_x,w)];
	}

	__syncthreads();

	if (j < new_h && i < new_w &&
			s_L[p] == PLATEAU && ghost == 0) {
		float I_p = tex2D(inputTexture,img_x,img_y);
		float I_q;
		int n_x, n_y; float L_q;

		for (int k = 0; k < 8; k++) {
			n_x = N_xs[k]+tx; n_y = N_ys[k]+ty;
			L_q = s_L[INDEX(n_y,n_x,BLOCK_SIZE)];
			if (L_q == INF || L_q >= 0) continue;
			int n_tx = L2I(i,n_x); int n_ty = L2I(j,n_y);
			int q = INDEX(n_ty,n_tx,w);
			I_q = tex2D(inputTexture,n_tx,n_ty);
			if (I_q == I_p && L[true_p] != -q) {
				L[true_p] = -q;
				atomicAdd(&C[0], 1);
				break;
			}
		}
	}
}

// Step 4.
__global__ void floodKernel(float* L, int* C, const int w, const int h)
{
	int i = blockDim.x * blockIdx.x + threadIdx.x;
	int j = blockDim.y * blockIdx.y + threadIdx.y;
	int p = INDEX(j,i,w); int q;

	if (j < h && i < w && L[p] <= 0) {
		q = -L[p];
		if (L[q] > 0 && L[p] != L[q]) {
			L[p] = L[q];
			atomicAdd(&C[0],1);
		}
	}
}

/**
 * This is an example method showing how to use this implementation.
 */
void segmentWithTextureAnalysis(const cv::Mat& input, cv::Mat& output) {

	// the type must be float with 1 channel
	if(!input.data || input.type()!= CV_32FC1) {
		cout << "Invalid input image, skipping process." << endl;
		return;
	}

	const size_t numElemts = input.rows * input.cols;
	const size_t sizeInBytes = numElemts * sizeof(float);

	// Allocate cudaArray and copy image data
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
	cudaArray *inputCuArray;
	SAFE_CALL(cudaMallocArray(&inputCuArray, &channelDesc, input.cols, input.rows), "CUDA Malloc Array Failed");
	SAFE_CALL(cudaMemcpyToArray(inputCuArray, 0, 0, input.ptr<float>(), sizeInBytes, cudaMemcpyHostToDevice), "CUDA Memcopy to array Failed");

	// Set texture parameters
	imageTexture.addressMode[0] = cudaAddressModeWrap;
	imageTexture.addressMode[1] = cudaAddressModeWrap;
	imageTexture.filterMode = cudaFilterModeLinear;
	imageTexture.normalized  = false;    // access with not normalized texture coordinates

	// Bind the array to the texture
	SAFE_CALL(cudaBindTextureToArray(imageTexture, inputCuArray, channelDesc), "cudaBindTextureToArray Failed");

	// Allocate device memory for labeled regions and initialize the memory to zero
	float *d_labeled;
	SAFE_CALL(cudaMalloc<float>(&d_labeled, numElemts * sizeof(float)), "CUDA Malloc Failed");
	SAFE_CALL(cudaMemset(d_labeled, 0, numElemts * sizeof(float)), "CUDA Memset Failed");

	// Allocate device memory for one integer, counter for labeled regions
	int *d_counters;
	SAFE_CALL(cudaMalloc<int>(&d_counters, sizeof(int)), "CUDA Malloc Failed");
	SAFE_CALL(cudaMemset(d_counters, 0, sizeof(int)), "CUDA Memset Failed");

	// Configure two default kernel sizes to efficiently use the shared memory
	dim3 blockDim6x6(6, 6, 1);
	dim3 gridDim6x6(ceil((float)input.cols/(blockDim6x6.x-2)), ceil((float)input.rows/(blockDim6x6.y-2)), 1);

	dim3 blockDim16x16(16, 16, 1);
	dim3 gridDim16x16(ceil((float)input.cols/(blockDim16x16.x-2)), ceil((float)input.rows/(blockDim16x16.y-2)), 1);


	// 1) apply the descent kernel

	descentKernel<<< gridDim6x6, blockDim6x6>>>(d_labeled, input.cols, input.rows);
	SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors


	// 2A) apply increment_kernel

	incrementKernel<<< gridDim6x6, blockDim6x6>>>(d_labeled, input.cols, input.rows);
	SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors

	SAFE_CALL(cudaMemset(d_counters, 0, sizeof(int)), "CUDA Memset Failed");
	int oldValue = -1, newValue = -2;

	while (oldValue != newValue) {
		oldValue = newValue;

		// 2B) apply increment_kernel
		minimaKernel<<< gridDim6x6, blockDim6x6>>>(d_labeled, d_counters, input.cols, input.rows);
		SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors

		SAFE_CALL(cudaMemcpy(&newValue, d_counters, sizeof(int), cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");
	}

	// 3) apply plateau kernel

	SAFE_CALL(cudaMemset(d_counters, 0, sizeof(int)), "CUDA Memset Failed");
	oldValue = -1;
	newValue = -2;

	while (oldValue != newValue) {
		oldValue = newValue;

		plateauKernel<<< gridDim6x6, blockDim6x6>>>(d_labeled, d_counters, input.cols, input.rows);
		SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors

		SAFE_CALL(cudaMemcpy(&newValue, d_counters, sizeof(int), cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");
	}


	// 4) apply flood kernel

	SAFE_CALL(cudaMemset(d_counters, 0, sizeof(int)), "CUDA Memset Failed");
	oldValue = -1;
	newValue = -2;

	while (oldValue != newValue) {
		oldValue = newValue;

		floodKernel<<< gridDim16x16, blockDim16x16>>>(d_labeled, d_counters, input.cols, input.rows);
		SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors

		SAFE_CALL(cudaMemcpy(&newValue, d_counters, sizeof(int), cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");
	}


	// Copy back data from destination device memory to OpenCV output image
	Mat markers(input.size(), input.type());  // CV_32FC1
	SAFE_CALL(cudaMemcpy(markers.ptr(), d_labeled, sizeInBytes, cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");

	// find the number of segments
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc( markers, &minVal, &maxVal, &minLoc, &maxLoc );
	int numOfSegments = maxVal; // markers contains region indices and mot pixel values

	// normalize the markers to have integer values between 0 and numOfSegments
	Mat normMarkers(input.size(), CV_32SC1);
	normalize(markers, normMarkers, 0, numOfSegments, NORM_MINMAX, CV_32SC1);

	// apply the regions over the original image
	Mat wshed = createSegmentationDisplay(normMarkers, numOfSegments, input);
	wshed.copyTo(output); // CV_32FC1

	// Unbind the device memory of the image to a texture variable
	SAFE_CALL(cudaUnbindTexture(inputTexture), "CudaBindTextureToArray Failed");

	//Free the device memory
	SAFE_CALL(cudaFreeArray(inputCuArray), "CUDA Free Failed");
	SAFE_CALL(cudaFree(d_labeled), "CUDA Free Failed");
	SAFE_CALL(cudaFree(d_counters), "CUDA Free Failed");
	//SAFE_CALL(cudaDeviceReset(),"CUDA Device Reset Failed");
}

#endif /* WATERSHED_ORDER_VARIANT */
