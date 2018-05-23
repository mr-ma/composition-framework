#!/usr/bin/env bash

clang -emit-llvm -S example.c
opt -load=../cmake-build-debug/src/libcomposition-framework.so -S < example.ll > /dev/null -constraint-analysis