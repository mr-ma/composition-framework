cmake_minimum_required(VERSION 3.1)

project(stlplus3 C CXX)

set(PROJECT_NAME stlplus)

add_library(stlplus STATIC
        digraph.hpp
        exceptions.hpp
        safe_iterator.hpp
        )
SET_TARGET_PROPERTIES(stlplus PROPERTIES LINKER_LANGUAGE CXX)