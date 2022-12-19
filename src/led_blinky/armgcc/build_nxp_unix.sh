#!/usr/bin/env bash

# Project root is one up from the bin directory.
PROJECT_ROOT=$LF_BIN_DIRECTORY/..


echo "starting K22F generation script into $LF_SOURCE_GEN_DIRECTORY"
echo "pwd is $(pwd)"

# Copy platform into /core
cp $PROJECT_ROOT/platform/lf_k22f_support.c $LF_SOURCE_GEN_DIRECTORY/core/platform/
cp $PROJECT_ROOT/platform/lf_k22f_support.h $LF_SOURCE_GEN_DIRECTORY/core/platform/
cp $PROJECT_ROOT/platform/platform.h $LF_SOURCE_GEN_DIRECTORY/core/
cp $PROJECT_ROOT/platform/reactor.c $LF_SOURCE_GEN_DIRECTORY/core/
cp $PROJECT_ROOT/platform/reactor_common.c $LF_SOURCE_GEN_DIRECTORY/core/

# Copy platform into /include/core
# TODO: Why are there two generated core dirs
cp $PROJECT_ROOT/platform/lf_k22f_support.c $LF_SOURCE_GEN_DIRECTORY/include/core/platform/
cp $PROJECT_ROOT/platform/lf_k22f_support.h $LF_SOURCE_GEN_DIRECTORY/include/core/platform/
cp $PROJECT_ROOT/platform/platform.h $LF_SOURCE_GEN_DIRECTORY/include/core/
cp $PROJECT_ROOT/platform/reactor.c $LF_SOURCE_GEN_DIRECTORY/include/core/
cp $PROJECT_ROOT/platform/reactor_common.c $LF_SOURCE_GEN_DIRECTORY/include/core/

# Copy project files
cp $PROJECT_ROOT/src/led_blinky/*.c $LF_SOURCE_GEN_DIRECTORY/
cp $PROJECT_ROOT/src/led_blinky/*.h $LF_SOURCE_GEN_DIRECTORY/

# Build project
cd $PROJECT_ROOT/src/led_blinky/armgcc

if [ -d "CMakeFiles" ];then rm -rf CMakeFiles; fi
if [ -f "Makefile" ];then rm -f Makefile; fi
if [ -f "cmake_install.cmake" ];then rm -f cmake_install.cmake; fi
if [ -f "CMakeCache.txt" ];then rm -f CMakeCache.txt; fi
cmake -DCMAKE_TOOLCHAIN_FILE="armgcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=debug  .
make -j 2>&1 | tee build_log.txt

