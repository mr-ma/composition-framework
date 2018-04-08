cmake_minimum_required(VERSION 3.5)
project(constraint-handler VERSION 0.1 LANGUAGES CXX)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.58 REQUIRED graph)
find_package(LLVM REQUIRED CONFIG)

add_library(ConstraintHandlerPass SHARED
            include/constraints/AnalysisPass.hpp
            include/constraints/ConflictGraph.hpp
            include/constraints/GraphPrinter.hpp
            include/constraints/GraphPass.hpp
            src/AnalysisPass.cpp
            src/ConflictGraph.cpp
            src/GraphPrinter.cpp
            src/GraphPass.cpp)

# LLVM is (typically) built with no C++ RTTI. We need to match that.
target_compile_options(ConstraintHandlerPass
                       PRIVATE
                       -fno-rtti)

target_include_directories(ConstraintHandlerPass
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
                           PRIVATE
                           ${Boost_INCLUDE_DIRS}
                           ${LLVM_INCLUDE_DIRS}
                           /usr/include/llvm-3.9/
                           /usr/include/llvm-c-3.9/)


target_link_libraries(ConstraintHandlerPass
                      PRIVATE
                      ${Boost_LIBRARIES})

target_compile_features(ConstraintHandlerPass
                        PUBLIC
                        cxx_deleted_functions)
# Get proper shared-library behavior (where symbols are not necessarily
# resolved when the shared library is linked) on OS X.
if (APPLE)
	target_link_libraries(ConstraintHandlerPass
	                      PRIVATE
	                      -undefined dynamic_lookup)
endif (APPLE)

INSTALL(
		DIRECTORY ${CMAKE_SOURCE_DIR}/include/
		DESTINATION include
		FILES_MATCHING PATTERN "*.h*")
install(TARGETS ConstraintHandlerPass EXPORT ConstraintHandlerPass
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
install(EXPORT ConstraintHandlerPass
        FILE ConstraintHandlerPassConfig.cmake
        NAMESPACE ConstraintHandlerPassConfig::
        DESTINATION lib/cmake/ConstraintHandlerPass)