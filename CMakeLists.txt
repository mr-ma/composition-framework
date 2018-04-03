cmake_minimum_required(VERSION 3.1)

set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
file(GLOB_RECURSE INCLUDE_DIR_HEADERS
     "${INCLUDE_DIR}/*.h"
     )
find_package(LLVM REQUIRED CONFIG)
add_definitions(${LLVM_DEFINITIONS})
include_directories(${LLVM_INCLUDE_DIRS})
link_directories(${LLVM_LIBRARY_DIRS})

if ($ENV{CLION_IDE})
	include_directories("/usr/include/llvm-3.9/")
	include_directories("/usr/include/llvm-c-3.9/")
endif ()
include_directories(${INCLUDE_DIR})

add_subdirectory(src)
