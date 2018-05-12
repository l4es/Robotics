/*
 * sobel_filter.cuh
 *
 *  Created on: Mar 21, 2015
 *      Author: claudiu
 */

#ifndef SOBEL_FILTER_CUH
#define SOBEL_FILTER_CUH

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

__device__ inline float3 multiplyCuda(const float a, const float3 b)
{
	return {a * b.x, a * b.y, a * b.z};
}

__device__ inline float3 addCuda(const float3 a, const float3 b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}

__device__ inline float3 toFloat3(const uchar3 a)
{
	return {(float)a.x, (float)a.y, (float)a.z};
}

__device__ inline uchar3 toUchar3 (const float3 a)
{
	return {(uchar)a.x, (uchar)a.y, (uchar)a.z};
}

// this method applies a gaussian kernel on d_input
// d_input and d_output are 3-channel data
__global__ void applyGaussianFilter(const uchar3* const d_input,
		const size_t width, const size_t height,
		const float delta, const int radius,
		uchar3* const d_output
)
{
	extern __shared__ float3 s_input[];

	//2D Index of current thread
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	//Only valid threads perform memory I/O
	if((x<width) && (y<height))
	{
		const int crtShareIndex = threadIdx.y * blockDim.x + threadIdx.x;
		const int crtGlobalIndex = y * width + x;
		s_input[crtShareIndex] = toFloat3(d_input[crtGlobalIndex]);
		__syncthreads();

		const int r = radius;
		float3 t = {0.f, 0.f, 0.f};
		float sum = 0.0f;
		float factor = 0.0f;

		for (int i = -r; i <= r; ++i)
		{
			int crtY = threadIdx.y + i; //clamp the neighbor pixel, prevent overflow
			if (crtY < 0)					crtY = 0;
			else if (crtY >= blockDim.y)   	crtY = blockDim.y - 1;

			for (int j = -r; j <= r; ++j)
			{
				int crtX = threadIdx.x + j;
				if (crtX < 0) 						crtX = 0;
				else if (crtX >= blockDim.x)	 	crtX = blockDim.x - 1;

				const float3 curPix = s_input[crtY * blockDim.x + crtX];
				factor = c_gaussian[r + i] * c_gaussian[r + j];
				sum += factor;
				t = addCuda(t, multiplyCuda(factor, curPix));
			}
		}

		d_output[y * width + x] = toUchar3(multiplyCuda(1.f / sum, t));
	}
}

// this method generates the histogram using the built-in atomics
// the input data is already normalized so it only applies this formula: histo[val[i]]++;
__global__ void convertToGrayscale(const uchar3* const d_input,
		const size_t width, const size_t height,
		uchar* const d_output
)
{
	//2D Index of current thread
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	const int idx = y * width + x;

	//Only valid threads perform memory I/O
	if((x<width) && (y<height))
	{
		const uchar3 imagePoint = d_input[idx];
		d_output[idx] = .299f*imagePoint.x + .587f*imagePoint.y  + .114f*imagePoint.z;
	}
}

const int sobel_width = 3;
__constant__ int c_sobel_x[sobel_width][sobel_width];   //sobel filter on the x axis
__constant__ int c_sobel_y[sobel_width][sobel_width];   //sobel filter on the y axis

// the kernel matrix is stored as array (1,r*r) instead of (r,r)
inline void setSobelKernels()
{
	const int numElements = sobel_width * sobel_width;
	const int h_sobel_x[sobel_width][sobel_width] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
	const int h_sobel_y[sobel_width][sobel_width] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
	SAFE_CALL( cudaMemcpyToSymbol(c_sobel_x, h_sobel_x, sizeof(int)*numElements), "CUDA Kernel Memcpy Host To Device Failed");
	SAFE_CALL( cudaMemcpyToSymbol(c_sobel_y, h_sobel_y, sizeof(int)*numElements), "CUDA Kernel Memcpy Host To Device Failed");
}

// this method applies both sobel filters for x and yon d_input and saves the combined result into d_output
// d_input and d_output are 1-channel data
__global__ void applySobelFilters(const uchar* const d_input,
		const size_t width, const size_t height,
		const int kernel_width,
		uchar* const d_output
)
{
	extern __shared__ uchar s_input2[];

	//2D Index of current thread
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	//Only valid threads perform memory I/O
	if((x<width) && (y<height))
	{
		const int crtShareIndex = threadIdx.y * blockDim.x + threadIdx.x;
		const int crtGlobalIndex = y * width + x;
		s_input2[crtShareIndex] = d_input[crtGlobalIndex];
		__syncthreads();

		const int r = (kernel_width - 1) / 2;
		int sum_x = 0;
		int sum_y = 0;

		for (int i = -r; i <= r; ++i)
		{
			int crtY = threadIdx.y + i; //clamp the neighbor pixel, prevent overflow
			if (crtY < 0)						crtY = 0;
			else if (crtY >= blockDim.y)   		crtY = blockDim.y - 1;

			for (int j = -r; j <= r; ++j)
			{
				int crtX = threadIdx.x + j;
				if (crtX < 0) 					crtX = 0;
				else if (crtX >= blockDim.x)	crtX = blockDim.x - 1;

				const float inputPix = (float)(s_input2[crtY * blockDim.x + crtX]);
				sum_x += inputPix * c_sobel_x[r + j][r + i];
				sum_y += inputPix * c_sobel_y[r + j][r + i];
			}
		}

		d_output[y * width + x] = (uchar) (abs(sum_x) + abs(sum_y));
	}
}

// this method does all operations required by the sobel filter:
// upload to GPU, convert to grayscale, gaussian filter and sobel operator, download from GPU
void sobelFilterCuda(const cv::Mat& input, cv::Mat& output)
{
	const size_t numElemts = output.rows * output.cols;

	//Allocate device memory
	uchar3 *d_input, *d_inputBlurred; // CV_U8C3
	uchar *d_inputGrayscale, *d_output; // CV_U8C1
	SAFE_CALL(cudaMalloc<uchar3>(&d_input, numElemts * sizeof(uchar3)), "CUDA Malloc Failed");
	SAFE_CALL(cudaMalloc<uchar3>(&d_inputBlurred, numElemts * sizeof(uchar3)), "CUDA Malloc Failed");
	SAFE_CALL(cudaMalloc<uchar>(&d_inputGrayscale, numElemts * sizeof(uchar)), "CUDA Malloc Failed");
	SAFE_CALL(cudaMalloc<uchar>(&d_output, numElemts * sizeof(uchar)), "CUDA Malloc Failed");

	//Copy data from OpenCV input image to device memory
	SAFE_CALL(cudaMemcpy(d_input, input.ptr<uchar3>(), numElemts * sizeof(uchar3), cudaMemcpyHostToDevice), "CUDA Memcpy Host To Device Failed");

	// set default kernel size as 16x16 to efficiently use the shared memory
	const dim3 blockDimHist(16, 16, 1);
	const dim3 gridDimHist(ceil((float)input.cols/blockDimHist.x), ceil((float)input.rows/blockDimHist.y), 1);
	size_t blockSharedMemory = 0;


	// 1) blur the input image using the gaussian filter to remove the noise

	// compute the gaussian kernel for the current radius and delta
	const float euclideanDelta = 1.0f;
	const int filterRadius = 3;
	computeGaussianKernelCuda(euclideanDelta, filterRadius);

	// apply the gaussian kernel
	blockSharedMemory = blockDimHist.x * blockDimHist.y * sizeof(float3);
	applyGaussianFilter<<< gridDimHist, blockDimHist, blockSharedMemory>>>(d_input, input.cols, input.rows,
			euclideanDelta, filterRadius, d_inputBlurred);
	SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors


	// 2) convert it to grayscale (CV_8UC3 -> CV_8UC1), +2ms
	convertToGrayscale<<< gridDimHist, blockDimHist, blockSharedMemory>>>(d_inputBlurred, input.cols, input.rows, d_inputGrayscale);
	SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors


	// 3) compute the gradients on both directions x and y, and combine the result into d_output

	// set the sobel kernels for both x and y axes and copy them to the constant memory for a faster memory access
	setSobelKernels();

	// apply the sobel kernels both for x and y at the same time
	blockSharedMemory = blockDimHist.x * blockDimHist.y * sizeof(uchar);
	applySobelFilters<<< gridDimHist, blockDimHist, blockSharedMemory>>>(d_inputGrayscale, input.cols, input.rows, sobel_width, d_output);
	SAFE_CALL(cudaDeviceSynchronize(), "Kernel Launch Failed"); //Synchronize to check for any kernel launch errors


	// Copy back data from destination device memory to OpenCV output image
	SAFE_CALL(cudaMemcpy(output.ptr(), d_output, numElemts * sizeof(uchar), cudaMemcpyDeviceToHost), "CUDA Memcpy Host To Device Failed");

	//Free the device memory
	SAFE_CALL(cudaFree(d_input), "CUDA Free Failed");
	SAFE_CALL(cudaFree(d_inputBlurred), "CUDA Free Failed");
	SAFE_CALL(cudaFree(d_inputGrayscale), "CUDA Free Failed");
	SAFE_CALL(cudaFree(d_output), "CUDA Free Failed");
	//SAFE_CALL(cudaDeviceReset(),"CUDA Device Reset Failed");
}

#endif /* SOBEL_FILTER_CUH */
