# README #

### Hello World CUDA ###

* This is a simple Hello World CUDA project which takes an array of integers, copies it to the GPU memory, computes the cube values of each element in parallel, copies back the array to the CPU memory and prints it in the console.

### How do I get set up? ###

* To start working with this project you must have cmake installed on your computer. 
Using cmake a Makefile will be generated, file which can be imported in an IDE such Eclipse.

To generate Makefile and to build it:
sh: cmake .
sh: make

To generate Makefile, .project and .cproject Debug version for Eclipse CDT:
sh: cd ..
sh: mkdir project-name_Debug
sh: cd project-name_Debug
sh: cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../project-name 

Details on how to import a CMake project in Eclipse:
https://cmake.org/Wiki/CMake:Eclipse_UNIX_Tutorial
https://cmake.org/Wiki/Eclipse_CDT4_Generator

For issues related to generating the Makefile because of the CUDA library:
http://bikulov.org/blog/2013/12/24/example-of-cmake-file-for-cuda-plus-cpp-code/

* The entire process of how to set it up and run on the NVIDIA Jetson Tegra TK1 development board is described in this post:
http://www.coldvision.io/2015/11/10/hello-world-cuda-program-for-jetson-tk1/

### Who do I talk to? ###

* claudiu }at{ coldvision.io
* http://www.coldvision.io
