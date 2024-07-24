#!/usr/bin/env bash
pushd ./ && \
cd ./pi_eink && \
  # rm -rf ./build/ && \
  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=clang++-18 -DCMAKE_BUILD_TYPE=Release -DBUILD_CLIENT=ON -B build . && \
  cmake --build build -j32 && \
  cmake --install build && \

cd ../comparator && \
  # rm -rf ./build/ && \
  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=clang++-18 -DCMAKE_BUILD_TYPE=Release -B build . && \
  cmake --build build -j32 && \
popd
