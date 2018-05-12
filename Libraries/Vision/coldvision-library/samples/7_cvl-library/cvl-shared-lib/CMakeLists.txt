cmake_minimum_required(VERSION 2.8.9)
project (TestCVL)

#For the shared library:
set ( PROJECT_LINK_LIBS libCVL.so )
link_directories( ../../build )
#link_directories( /usr/lib )

#For the static library:
#set ( PROJECT_LINK_LIBS libtestStudent.a )
#link_directories( ../../build )

include_directories( 
	../../modules/core/include
	../../modules/imgproc/include 
	../../modules/ml/include )

add_executable(TestCVL main.cpp)
target_link_libraries(TestCVL ${PROJECT_LINK_LIBS} )
