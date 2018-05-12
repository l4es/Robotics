## Settings for cross-compiling the RDK to a Gumstix system
#
# This file needs to be included in your manual.cmake:
#
#   INCLUDE(build_tools/cmake/Gumstix.cmake)
#
# Before running cmake, you need to:
#
#  - include the packet "libxml2" into the Gumstix buildroot
#
#  - cross-compile libgsl and install it into the
#    buildroot. Instructions for cross-compiling are in the wiki.
#
#  - set the following variables to 0 inside manual.cmake:
#     - COMPILE_GMAPPING 
#     - COMPILE_RCONSOLE
#     - COMPILE_CONFIGBUILDER
#
# Paths to the buildroot in this file are based on the latest version
#

MESSAGE(STATUS " *** Building for Gumstix *** ")
SET(GUMSTIX 1)

# this one is important
SET(CMAKE_SYSTEM Linux)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER   $ENV{GUMSTIX_BUILDROOT}/build_arm_nofpu/staging_dir/bin/arm-linux-gcc)
SET(CMAKE_CXX_COMPILER $ENV{GUMSTIX_BUILDROOT}/build_arm_nofpu/staging_dir/bin/arm-linux-g++)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  $ENV{GUMSTIX_BUILDROOT}/build_arm_nofpu/root)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Where to find GSL: in the Gumstix root filesystem.
SET($ENV{GSL_HOME} ${CMAKE_FIND_ROOT_PATH}/usr)

# HACK: tell we have XML2 libraries, but with the local include files
# this is because the Gumstix distro does not include headers
SET(LIBXML_FOUND 1)
SET(LIBXML2_INCLUDE_DIR $ENV{GUMSTIX_BUILDROOT}/build_arm_nofpu/staging_dir/usr/include/libxml2)
SET(LIBXML2_LIBRARIES $ENV{GUMSTIX_BUILDROOT}/build_arm_nofpu/root/usr/lib/libxml2.so)

# Set the RDK root directory. Please note that this must be relative
# to the Gumstix root!
SET(RDK2_ROOT /root/rdk2)
