/*
 */
#ifndef WATERSHED_UTILS
#define WATERSHED_UTILS

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

#endif /* WATERSHED_UTILS */
