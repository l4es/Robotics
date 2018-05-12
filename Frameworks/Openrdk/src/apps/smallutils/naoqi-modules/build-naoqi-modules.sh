#!/bin/bash - 
#===============================================================================
#
#          FILE:  build-naoqi-modules.sh
# 
#         USAGE:  ./build-naoqi-modules.sh 
# 
#   DESCRIPTION:  
# 
#       OPTIONS:  ---
#  REQUIREMENTS:  ---
#          BUGS:  ---
#         NOTES:  ---
#        AUTHOR: Luca Marchetti (LM), <luca.marchetti@cipicchia.net>
#       COMPANY: 
#       CREATED: 03/03/10 22:07:22 CET
#      REVISION:  ---
#===============================================================================
set -o nounset                              # Treat unset variables as an error

mkdir -p build-linux
cd build-linux
cmake -DCMAKE_TOOLCHAIN_FILE=${AL_DIR}/toolchain-pc.cmake ..
#ccmake .
make

