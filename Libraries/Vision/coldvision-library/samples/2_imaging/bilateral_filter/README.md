# README #

### What is this repository for? ###

*   This prototype tests different implementations of the bilateral filtering to smooth images using C++, CUDA, OpenCV 3.X.

Several smoothing algorithms exist, but the most popular are:
*   Normalized Box Filter: this filter is the simplest of all. Each output pixel is the mean of its kernel neighbors (all of them contribute with equal weights)
*   Gaussian Filter: Probably the most useful filter (although not the fastest). Gaussian filtering is done by convolving each point in the input array with a Gaussian kernel and then summing them all to produce the output array.
*   Median Filter: The median filter run through each element of the signal (in this case the image) and replace each pixel with the median of its neighboring pixels (located in a square neighborhood around the evaluated pixel).
*   Bilateral Filter: In an analogous way as the Gaussian filter, the bilateral filter also considers the neighboring pixels with weights assigned to each of them. These weights have two components, the first of which is the same weighting used by the Gaussian filter. The second component takes into account the difference in intensity between the neighboring pixels and the evaluated one.

A bilateral filter is a non-linear, edge-preserving and noise-reducing smoothing filter for images. The intensity value at each pixel in an image is replaced by a weighted average of intensity values from nearby pixels. This weight can be based on a Gaussian distribution. Crucially, the weights depend not only on Euclidean distance of pixels, but also on the radiometric differences (e.g. range differences, such as color intensity, depth distance, etc.). This preserves sharp edges by systematically looping through each pixel and adjusting weights to the adjacent pixels accordingly.

Four different methods are compared to each other in this prototype:
*   OpenCV 3.x CPU based method cv::blur from the imgproc module.
*   OpenCV 3.x GPU based method cv::cuda::bilateralFilter() from the cudaimgproc module.
*   Own method bilateralFilterCpu() implemented in C++ to run in serial on the CPU.
*   Own method bilateralFilterCuda() implemented in Cuda to run in parallel on the GPU.

Appart from the 4 methods above the cuda toolkit provides a more advanced example which can be found under toolkit/samples/3_imaging/bilateralFilter or online an older version at https://www.ecse.rpi.edu/~wrf/wiki/ParallelComputingSpring2014/cuda-samples/samples/3_Imaging/bilateralFilter/.

### How do I get set up? ###

*   Details can be found in this post:
http://www.coldvision.io/2016/01/21/bilateral-filtering-with-cuda-and-opencv-3-x/

### Who do I talk to? ###

*   claudiu }at{ coldvision.io
*   http://www.coldvision.io
