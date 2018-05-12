/*
 * bilateral_filter_cpu.h
 *
 *  Created on: Jan 3, 2016
 *      Author: claudiu
 */
#ifndef BILATERAL_FILTER_H_
#define BILATERAL_FILTER_H_

#include <math.h>

////////////////////////////////////////////////////////////////////////////////

// export C interface
extern "C" void computeGaussianKernel(const float delta, const int radius);

extern "C" void bilateralFilterCpu(const float4 * const src,
		float4 * const dest,
		const float euclidean_delta,
		const int width, const int height,
		const int filter_radius);

float gaussian[64];

// define an array of size 2 * radius + 1 to be used for both axis
inline void computeGaussianKernel(const float delta, const int radius)
{
	for (int i = 0; i < 2 * radius + 1; ++i)
	{
		const float x = i - radius;
		gaussian[i] = expf( -(x * x) / (2.0f * delta * delta) );
	}
}

// it computes the euclidean distance between two points, each point a vector with 4 elements
inline float euclideanLen(const float4 a, const float4 b, const float d)
{
	const float mod = (b.x - a.x) * (b.x - a.x) +
			(b.y - a.y) * (b.y - a.y) +
			(b.z - a.z) * (b.z - a.z) +
			(b.w - a.w) * (b.w - a.w);
	return expf(-mod / (2.0f * d * d));
}

inline float4 multiply(const float a, const float4 b)
{
	return {a * b.x, a * b.y, a * b.z, a * b.w};
}

inline float4 add(const float4 a, const float4 b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

void bilateralFilterCpu(const float4 * const src,
		float4 * const dest,
		const float euclidean_delta,
		const int width, const int height,
		const int filter_radius)
{
	computeGaussianKernel(euclidean_delta, filter_radius);

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			float sum = 0.0f;
			float4 t = {0.f, 0.f, 0.f, 0.f};
			const float4 center = src[y * width + x];
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

					const float4 curPix = src[crtY * width + crtX];
					domainDist = gaussian[r + i] * gaussian[r + j];
					colorDist = euclideanLen(curPix, center, euclidean_delta);
					factor = domainDist * colorDist;
					sum += factor;
					t = add(t, multiply(factor, curPix));
				}
			}

			dest[y * width + x] = multiply(1.f / sum, t);
		}
	}
}
#endif /* BILATERAL_FILTER_H_ */
