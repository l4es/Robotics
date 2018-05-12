#!/bin/bash - 
#===============================================================================
#
#          FILE:  buildcross-naoqi-modules.sh
# 
#         USAGE:  ./buildcross-naoqi-modules.sh 
# 
#   DESCRIPTION:  
# 
#       OPTIONS:  ---
#  REQUIREMENTS:  ---
#          BUGS:  ---
#         NOTES:  ---
#        AUTHOR: Luca Marchetti (LM), <luca.marchetti@cipicchia.net>
#       COMPANY: 
#       CREATED: 20/04/10 12:07:22 CET
#      REVISION:  ---
#===============================================================================
set -o nounset                              # Treat unset variables as an error

mkdir -p build-nao
cd build-nao
cmake -DCMAKE_TOOLCHAIN_FILE=${OE_CROSS_DIR}/toolchain-geode.cmake ..
#ccmake .
make

