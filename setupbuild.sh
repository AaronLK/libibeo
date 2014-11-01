#!/bin/bash

#BUILD_TOOL="CodeBlocks - Unix Makefiles"
BUILD_TOOL="CodeBlocks - Ninja"

mkdir build
cd build
rm -rf debug/* release/*
mkdir debug
mkdir release
cd debug
cmake -G "$BUILD_TOOL" ../../ -DCMAKE_BUILD_TYPE=Debug
cd ../release
cmake -G  "$BUILD_TOOL" ../../ -DCMAKE_BUILD_TYPE=Release 

