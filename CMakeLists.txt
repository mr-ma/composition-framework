cmake_minimum_required(VERSION 3.12)
project(composition-framework VERSION 0.4.0 LANGUAGES CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

##
## Config
##
set(CompositionFramework_BuildTests "OFF" CACHE BOOLEAN "")
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

##
## Dependencies
##
find_package(Boost 1.58 REQUIRED)
find_package(LLVM 7.0 REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED)
find_package(LEMON REQUIRED)
find_package(function-filter REQUIRED)
find_package(OpenMP REQUIRED)

##
## CONFIGURATION
##
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)

##
## TARGET
## create target and add include path
##

add_library(CompositionFramework SHARED
        include/composition/graph/ProtectionGraph.hpp
        include/composition/graph/edge.hpp
        include/composition/graph/vertex.hpp

        include/composition/graph/algorithm/all_cycles.hpp

        include/composition/graph/constraint/bitmask.hpp
        include/composition/graph/constraint/constraint.hpp
        include/composition/graph/constraint/dependency.hpp
        include/composition/graph/constraint/present.hpp
        include/composition/graph/constraint/preserved.hpp
        include/composition/graph/constraint/true.hpp

        include/composition/graph/util/dot.hpp
        include/composition/graph/util/graphml.hpp

        include/composition/metric/Coverage.hpp
        include/composition/metric/Performance.hpp
        include/composition/metric/Weights.hpp
        include/composition/metric/Stats.hpp
        include/composition/metric/Connectivity.hpp
        include/composition/metric/ManifestStats.hpp

        include/composition/trace/TraceableValue.hpp
        include/composition/trace/PreservedValueRegistry.hpp

        include/composition/support/Analysis.hpp
        include/composition/AnalysisRegistry.hpp
        include/composition/AnalysisRegistry.hpp
        include/composition/CompositionFrameworkPass.hpp
        include/composition/Manifest.hpp
        include/composition/support/ManifestValueHandle.hpp
        include/composition/ManifestRegistry.hpp
        include/composition/support/Pass.hpp
        include/composition/support/options.hpp
        include/composition/Stats.hpp
        include/composition/profiler.hpp

        include/composition/util/functions.hpp
        include/composition/util/strings.hpp
        include/composition/util/bimap.hpp

        src/composition/graph/vertex.cpp
        src/composition/graph/edge.cpp
        src/composition/graph/ProtectionGraph.cpp

        src/composition/graph/constraint/constraint.cpp
        src/composition/graph/constraint/dependency.cpp
        src/composition/graph/constraint/present.cpp
        src/composition/graph/constraint/preserved.cpp
        src/composition/graph/constraint/true.cpp

        src/composition/metric/Coverage.cpp
        src/composition/metric/Performance.cpp
        src/composition/metric/Weights.cpp
        src/composition/metric/Stats.cpp
        src/composition/metric/Connectivity.cpp

        src/composition/trace/TraceableValue.cpp
        src/composition/trace/PreservedValueRegistry.cpp

        src/composition/util/functions.cpp
        src/composition/util/strings.cpp

        src/composition/CompositionFrameworkPass.cpp
        src/composition/ManifestRegistry.cpp
        src/composition/support/options.cpp
        src/composition/Manifest.cpp
        src/composition/support/Pass.cpp
        src/composition/AnalysisRegistry.cpp
        src/composition/Stats.cpp
        src/composition/support/ManifestValueHandle.cpp)

add_library(composition-framework::CompositionFramework ALIAS CompositionFramework)

add_library(rtlib SHARED
        hook/hook.cpp)

add_library(composition-framework::CompositionFrameworkHook ALIAS rtlib)

target_compile_options(rtlib PRIVATE -g)

target_include_directories(CompositionFramework
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${Boost_INCLUDE_DIRS}
        ${LLVM_INCLUDE_DIRS}
        ${LEMON_INCLUDE_DIRS}
        )

if ($ENV{CLION_IDE})
    include_directories("/usr/include/llvm-7.0/")
    include_directories("/usr/include/llvm-c-7.0/")
endif ()


target_compile_options(CompositionFramework PRIVATE -g -fno-rtti)
target_compile_features(CompositionFramework PUBLIC cxx_std_17)
target_link_libraries(CompositionFramework PRIVATE glpk emon nlohmann_json::nlohmann_json OpenMP::OpenMP_CXX)

target_compile_features(rtlib PUBLIC cxx_std_17)

##
## INSTALL
## install header files, generate and install cmake config files for find_package()
##
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/composition)

install(
        TARGETS CompositionFramework
        EXPORT composition-framework-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
        EXPORT composition-framework-targets
        NAMESPACE composition-framework::
        DESTINATION ${INSTALL_CONFIGDIR}
)

##
## Config Version file
##
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/composition-frameworkConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/composition-frameworkConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/composition-frameworkConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/composition-frameworkConfigVersion.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/composition-frameworkConfig.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
        )


##
## Export
##
export(
        EXPORT composition-framework-targets
        NAMESPACE composition-framework::
        FILE ${CMAKE_CURRENT_BINARY_DIR}/composition-frameworkTargets.cmake
)

export(PACKAGE composition-framework)


##
## Test
##
if (CompositionFramework_BuildTests)
    add_subdirectory(test)
endif ()
