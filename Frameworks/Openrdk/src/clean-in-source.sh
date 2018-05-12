find . -iname Makefile | xargs rm
find . -iname CMakeCache.txt | xargs rm
find . -iname CMakeFiles | xargs rm -rf
find . -iname CPackConfig.cmake | xargs rm
find . -iname CPackSourceConfig.cmake | xargs rm
find . -iname CTestTestfile.cmake | xargs rm
find . -iname cmake_install.cmake | xargs rm
rm cross.cmake FindOpenRDK.cmake manual.cmake openrdk_manifest.txt setenv
rm -rf ../lib ../libext ../bin ../binext
