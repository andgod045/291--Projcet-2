#!/bin/zsh
rm -rf build
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=xc32-toolchain.cmake
cmake --build build