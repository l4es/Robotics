/*
 * bilateral_filter_cuda.cuh
 *
 *  Created on: Jan 3, 2016
 *      Author: claudiu
 */

#ifndef BILATERAL_FILTER_CUH_
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

// export C interface
extern "C" void computeGaussianKernelCuda(const float delta, const int radius);

extern "C" void bilateralFilterCuda(const float4 * const h_input,
		float4 * const h_output,
		const float euclidean_delta,
		const int width, const int height,
		const int filter_radius);

__constant__ float c_gaussian[64];   //gaussian array in device side

// it uses only one axis of the kernel (1,2r) instead of a matrix (2r,2r)
inline void computeGaussianKernelCuda(const float delta, const int radius)
{
	float h_gaussian[64];
	for (int i = 0; i < 2 * radius + 1; ++i)
	{
		const float x = i - radius;
		h_gaussian[i] = expf( -(x * x) / (2.0f * delta * delta) );
	}
	SAFE_CALL( cudaMemcpyToSymbol(c_gaussian, h_gaussian, sizeof(float)*(2*radius+1)), "CUDA Kernel Memcpy Host To Device Failed");
}

// it computes the euclidean distance between two points, each point a vector with 4 elements
__device__ inline float euclideanLenCuda(const float4 a, const float4 b, const float d)
{
	const float mod = (b.x - a.x) * (b.x - a.x) +
			(b.y - a.y) * (b.y - a.y) +
			(b.z - a.z) * (b.z - a.z) +
			(b.w - a.w) * (b.w - a.w);
	return expf(-mod / (2.0f * d * d));
}

__device__ inline float4 multiplyCuda(const float a, const float4 b)
{
	return {a * b.x, a * b.y, a * b.z, a * b.w};
}

__device__ inline float4 addCuda(const float4 a, const float4 b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

__global__ void bilateralFilterCudaKernel( const float4 * const d_input,
		float4 * const d_output,
		const float euclidean_delta,
		const int width, const int height,
		const int filter_radius)
{
	//2D Index of current thread
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	//Only valid threads perform memory I/O
	if((x<width) && (y<height))
	{
		float sum = 0.0f;
		float4 t = {0.f, 0.f, 0.f, 0.f};
		const float4 center = d_input[y * width + x];
		const int r = filter_radius;

		float domainDist=0.0f, colorDist=0.0f, factor=0.0f;

		for (int i = -r; i <= r; ++i)
		{
			int crtY = y + i; //clamp the neighbor pixel, prevent overflow
			if (crtY < 0)				crtY = 0;
			else if (crtY >= height)   	crtY = height - 1;

			for (int j = -r; j <= r; ++j)
			{
				int crtX = x + j;
				if (crtX < 0) 				crtX = 0;
				else if (crtX >= width)	 	crtX = width - 1;

				const float4 curPix = d_input[crtY * width + crtX];
				domainDist = c_gaussian[r + i] * c_gaussian[r + j];
				colorDist = euclideanLenCuda(curPix, center, euclidean_delta);
				factor = domainDist * colorDist;
				sum += factor;
				t = addCuda(t, multiplyCuda(factor, curPix));
			}
		}

		d_output[y * width + x] = multiplyCuda(1.f / sum, t);
	}
}

void bilateralFilterCuda(const float4 * const h_input,
		float4 * const h_output,
		const float euclidean_delta,
		const int width, const int height,
		const int filter_radius)
{
	// compute the gaussian kernel for the current radius and delta
	computeGaussianKernelCuda(euclidean_delta, filter_radius);

	// copy the input image from the CPU´s memory to the GPU´s global memory
	const int inputBytes = width * height * sizeof(float4);
	const int outputBytes = inputBytes;
	float4 *d_input, *d_output; // arrays in the GPU´s global memory
	// allocate device memory
	SAFE_CALL(cudaMalloc<float4>(&d_input, inputBytes), "CUDA Malloc Failed");
	SAFE_CALL(cudaMalloc<float4>(&d_output, outputBytes), "CUDA Malloc Failed");
	// copy data of input image to device memory
	SAFE_CALL(cudaMemcpy(d_input, h_input, inputBytes, cudaMemcpyHostToDevice), "CUDA Memcpy Host To Device Failed");

	//GpuTimer timer;
	//timer.Start();

	// specify a reasonable grid and block sizes
	const dim3 block(16,16);
	// calculate grid size to cover the whole image
	const dim3 grid((width + block.x - 1)/block.x, (height + block.y - 1)/block.y);

	// launch the size conversion kernel
	bilateralFilterCudaKernel<<<grid,block>>>(d_input, d_output, euclidean_delta, width, height, filter_radius);

	//timer.Stop();
	//printf("Own Cuda code ran in: %f msecs.\n", timer.Elapsed());

	// synchronize to check for any kernel launch errors
	SAFE_CALL(cudaDeviceSynchronize(),"Kernel Launch Failed");

	//Copy back data from destination device meory to OpenCV output image
	SAFE_CALL(cudaMemcpy(h_output, d_output, outputBytes, cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");

	//Free the device memory
	SAFE_CALL(cudaFree(d_input),"CUDA Free Failed");
	SAFE_CALL(cudaFree(d_output),"CUDA Free Failed");
	//SAFE_CALL(cudaDeviceReset(),"CUDA Device Reset Failed");
}

#endif /* BILATERAL_FILTER_CUH_ */
