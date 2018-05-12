# cmake needs this line
cmake_minimum_required(VERSION 2.8.9)

# Define project name
project(CVL)

# Configure the build to print all details.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Only available from version 2.8.9 on
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

# Configure the build type. This can be passed as parameter too.
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Bring the headers, *.hpp into the project
include_directories(include)
include_directories(modules/core/include)
include_directories(modules/imgproc/include)
include_directories(modules/ml/include)


# Bring the sources, *.cpp into the project
#set(SOURCES ./src/main.cpp)
file(GLOB SOURCES "./modules/*.cpp")

# Can build statically to libloldvision-library.a using the next line
#add_library(CVL STATIC ${SOURCES})

# Building shared library to libEBBLibrary.so using the next line
add_library(CVL SHARED ${SOURCES})

target_link_libraries(CVL ${CMAKE_THREAD_LIBS_INIT})

install (TARGETS CVL DESTINATION /usr/lib)



