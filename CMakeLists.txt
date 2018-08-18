cmake_minimum_required(VERSION 3.8)
project(composition-framework VERSION 0.2 LANGUAGES CXX)


##
## INCLUDE
##
##
include(ExternalProject)

##
## OPTIONS
##


##
## CONFIGURATION
##
set(COMPOSITION_FRAMEWORK_TARGET_NAME ${PROJECT_NAME})
set(COMPOSITION_FRAMEWORK_CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}" CACHE INTERNAL "")
set(COMPOSITION_FRAMEWORK_INCLUDE_INSTALL_DIR "include")
set(COMPOSITION_FRAMEWORK_TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(COMPOSITION_FRAMEWORK_CMAKE_CONFIG_TEMPLATE "cmake/config.cmake.in")
set(COMPOSITION_FRAMEWORK_CMAKE_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(COMPOSITION_FRAMEWORK_CMAKE_VERSION_CONFIG_FILE "${COMPOSITION_FRAMEWORK_CMAKE_CONFIG_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(COMPOSITION_FRAMEWORK_CMAKE_PROJECT_CONFIG_FILE "${COMPOSITION_FRAMEWORK_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Config.cmake")
set(COMPOSITION_FRAMEWORK_CMAKE_PROJECT_TARGETS_FILE "${COMPOSITION_FRAMEWORK_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Targets.cmake")
set(COMPOSITION_FRAMEWORK_INCLUDE_BUILD_DIR "${PROJECT_SOURCE_DIR}/include/")


##
## Dependencies
##
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.58 REQUIRED graph)
find_package(LLVM 6.0 REQUIRED CONFIG)
##
## TARGET
## create target and add include path
##

add_library(${COMPOSITION_FRAMEWORK_TARGET_NAME} MODULE
        include/composition/filter/FunctionFilter.hpp
        include/composition/filter/FunctionInfo.hpp
        include/composition/filter/ValueInfo.hpp
        include/composition/filter/FilterInfo.hpp

        include/composition/graph/graph.hpp
        include/composition/graph/ProtectionGraph.hpp
        include/composition/graph/constraint.hpp
        include/composition/graph/edge.hpp
        include/composition/graph/edge_type.hpp
        include/composition/graph/vertex.hpp
        include/composition/graph/vertex_type.hpp

        include/composition/graph/algorithm/strong_components.hpp
        include/composition/graph/algorithm/topological_sort.hpp

        include/composition/graph/filter/filter.hpp
        include/composition/graph/filter/removed.hpp
        include/composition/graph/filter/dependency.hpp
        include/composition/graph/filter/selfcycle.hpp

        include/composition/graph/util/dot.hpp
        include/composition/graph/util/graphviz.hpp
        include/composition/graph/util/graphml.hpp
        include/composition/graph/util/index_map.hpp
        include/composition/graph/util/vertex_count.hpp
        include/composition/graph/util/constraint_map.hpp

        include/composition/metric/Coverage.hpp
        include/composition/metric/Performance.hpp

        include/composition/trace/TraceableValue.hpp
        include/composition/trace/PreservedValueRegistry.hpp

        include/composition/Analysis.hpp
        include/composition/AnalysisPass.hpp
        include/composition/AnalysisRegistry.hpp
        include/composition/AnalysisRegistry.hpp
        include/composition/GraphPass.hpp
        include/composition/Manifest.hpp
        include/composition/ManifestRegistry.hpp
        include/composition/Pass.hpp
        include/composition/Protection.hpp
        include/composition/ProtectionPass.hpp
        include/composition/ProtectionRegistry.hpp

        include/composition/ConflictResolver.hpp
        include/composition/ConflictResolverRegistry.hpp
        include/composition/EnhancementResolver.hpp
        include/composition/EnhancementResolverRegistry.hpp

        include/composition/util/functions.hpp
        include/composition/util/strings.hpp
        include/composition/util/enums.hpp

        src/filter/FunctionFilter.cpp

        src/graph/vertex.cpp
        src/graph/vertex_type.cpp
        src/graph/edge.cpp
        src/graph/edge_type.cpp
        src/graph/ProtectionGraph.cpp
        src/graph/constraint.cpp

        src/graph/util/graphviz.cpp

        src/metric/Coverage.cpp
        src/metric/Performance.cpp

        src/trace/TraceableValue.cpp
        src/trace/PreservedValueRegistry.cpp

        src/util/functions.cpp
        src/util/strings.cpp

        src/AnalysisPass.cpp
        src/GraphPass.cpp
        src/ProtectionPass.cpp
        src/ManifestRegistry.cpp)

add_library(${PROJECT_NAME}::${COMPOSITION_FRAMEWORK_TARGET_NAME} ALIAS ${COMPOSITION_FRAMEWORK_TARGET_NAME})
target_compile_features(${COMPOSITION_FRAMEWORK_TARGET_NAME} PUBLIC cxx_std_17)


target_include_directories(
        ${COMPOSITION_FRAMEWORK_TARGET_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${COMPOSITION_FRAMEWORK_INCLUDE_BUILD_DIR}>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${LLVM_INCLUDE_DIRS}
)

if ($ENV{CLION_IDE})
    include_directories("/usr/include/llvm-6.0/")
endif ()

# LLVM is (typically) built with no C++ RTTI. We need to match that.
target_compile_options(${COMPOSITION_FRAMEWORK_TARGET_NAME}
        PRIVATE
        -g)

target_link_libraries(${COMPOSITION_FRAMEWORK_TARGET_NAME}
        PRIVATE
        ${Boost_LIBRARIES})
##
## INSTALL
## install header files, generate and install cmake config files for find_package()
##
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${COMPOSITION_FRAMEWORK_CMAKE_VERSION_CONFIG_FILE} COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
        ${COMPOSITION_FRAMEWORK_CMAKE_CONFIG_TEMPLATE}
        ${COMPOSITION_FRAMEWORK_CMAKE_PROJECT_CONFIG_FILE}
        INSTALL_DESTINATION ${COMPOSITION_FRAMEWORK_CONFIG_INSTALL_DIR}
)

install(
        DIRECTORY ${COMPOSITION_FRAMEWORK_INCLUDE_BUILD_DIR}
        DESTINATION ${COMPOSITION_FRAMEWORK_INCLUDE_INSTALL_DIR}
)
install(
        FILES ${COMPOSITION_FRAMEWORK_CMAKE_PROJECT_CONFIG_FILE} ${COMPOSITION_FRAMEWORK_CMAKE_VERSION_CONFIG_FILE}
        DESTINATION ${COMPOSITION_FRAMEWORK_CONFIG_INSTALL_DIR}
)

export(
        TARGETS ${COMPOSITION_FRAMEWORK_TARGET_NAME}
        NAMESPACE ${PROJECT_NAME}::
        FILE ${COMPOSITION_FRAMEWORK_CMAKE_PROJECT_TARGETS_FILE}
)
install(
        TARGETS ${COMPOSITION_FRAMEWORK_TARGET_NAME}
        EXPORT ${COMPOSITION_FRAMEWORK_TARGETS_EXPORT_NAME}
        INCLUDES DESTINATION ${COMPOSITION_FRAMEWORK_INCLUDE_INSTALL_DIR}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        EXPORT ${COMPOSITION_FRAMEWORK_TARGETS_EXPORT_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${COMPOSITION_FRAMEWORK_CONFIG_INSTALL_DIR}
)
