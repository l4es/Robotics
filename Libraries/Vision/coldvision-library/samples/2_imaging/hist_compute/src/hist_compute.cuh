/*
 * resize.cuh
 *
 *  Created on: Dec 4, 2015
 *      Author: claudiu
 */

#ifndef RESIZE_CUH_
#define BILATERAL_FILTER_CUH_

#include<iostream>
#include<cstdio>
#include<cuda_runtime.h>

using std::cout;
using std::endl;

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

// this method generates the histogram using the built-in atomics
// the input data is already normalized so it only applies this formula: histo[val[i]]++;
__global__ void apply_gaussian_filter(const unsigned char* const d_input,
		const size_t numElems,
		unsigned int* const d_histogram
)
{
	const int d_1D_pos = blockIdx.x * blockDim.x + threadIdx.x;
	if (d_1D_pos < numElems) {
		const int myBin = d_input[d_1D_pos];
		atomicAdd(&(d_histogram[myBin]), 1);
	}
}

// this method computes the histogram of a single color channel with values between 0-255
void calcHistCuda(const cv::Mat& input, cv::Mat& output)
{
	const unsigned int numElems = input.rows * input.cols;

	//Calculate total number of bytes of input and output image
	const int inputBytes = input.step * input.rows;
	const int outputBytes = output.step * output.rows;

	unsigned char *d_input;
	unsigned int *d_output;

	//Allocate device memory
	SAFE_CALL(cudaMalloc<unsigned char>(&d_input,inputBytes),"CUDA Malloc Failed");
	SAFE_CALL(cudaMalloc<unsigned int>(&d_output,outputBytes),"CUDA Malloc Failed");

	//Copy data from OpenCV input image to device memory
	SAFE_CALL(cudaMemcpy(d_input, input.ptr(), inputBytes, cudaMemcpyHostToDevice), "CUDA Memcpy Host To Device Failed");

	GpuTimer timer;
	timer.Start();

	// generate the histogram using the built-in atomics
	const dim3 blockDimHist(256, 1, 1);
	const dim3 gridDimHist(ceil((float)numElems/(blockDimHist.x)), 1, 1);
	apply_gaussian_filter<<< gridDimHist, blockDimHist, 0>>>(d_input, numElems, d_output);

	timer.Stop();
	printf("Own Cuda code ran in: %f msecs.\n", timer.Elapsed());

	//Synchronize to check for any kernel launch errors
	SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed");

	//Copy back data from destination device meory to OpenCV output image
	SAFE_CALL(cudaMemcpy(output.ptr(), d_output, outputBytes, cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");

	//Free the device memory
	SAFE_CALL(cudaFree(d_input), "CUDA Free Failed");
	SAFE_CALL(cudaFree(d_output), "CUDA Free Failed");
	//SAFE_CALL(cudaDeviceReset(),"CUDA Device Reset Failed");
}

#endif /* RESIZE_CUH_ */
