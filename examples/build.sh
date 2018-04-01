#!/usr/bin/env bash

clang -emit-llvm -S example.c
opt -load=../cmake-build-debug/constraint-handler/libConstraintHandlerPass.so -S < example.ll > /dev/null -constraint-analysis