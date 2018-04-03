cmake_minimum_required(VERSION 3.1)

project(stlplus C CXX)

set(PROJECT_NAME stlplus)

add_library(stlplus STATIC
            digraph.hpp
            exceptions.hpp
            safe_iterator.hpp
            )

set_target_properties(stlplus PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(stlplus PROPERTIES CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set_target_properties(stlplus PROPERTIES CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")