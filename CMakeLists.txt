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
set(COMPOSITION_FRAMEWORK_CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}"
    CACHE INTERNAL "")
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
find_package(LLVM REQUIRED CONFIG)
##
## TARGET
## create target and add include path
##

add_library(${COMPOSITION_FRAMEWORK_TARGET_NAME} MODULE
            include/composition/filter/FunctionFilter.hpp
            include/composition/filter/FunctionInfo.hpp
            include/composition/filter/ValueInfo.hpp
            include/composition/filter/FilterInfo.hpp

            include/composition/graph/vertex_type.hpp
            include/composition/graph/vertex_attribute_set.hpp
            include/composition/graph/edge_type.hpp
            include/composition/graph/dot.hpp
            include/composition/graph/empty_graph.hpp
            include/composition/graph/graph.hpp
            include/composition/graph/ProtectionGraph.hpp
            include/composition/graph/protection.hpp

            include/composition/trace/TraceableValue.hpp
            include/composition/trace/PreservedValueRegistry.hpp

            include/composition/Analysis.hpp
            include/composition/AnalysisPass.hpp
            include/composition/AnalysisRegistry.hpp
            include/composition/AnalysisRegistry.hpp
            include/composition/GraphPrinter.hpp
            include/composition/GraphPass.hpp
            include/composition/Manifest.hpp
            include/composition/ManifestRegistry.hpp
            include/composition/Pass.hpp
            include/composition/Protection.hpp
            include/composition/ProtectionPass.hpp
            include/composition/ProtectionRegistry.hpp
            include/composition/util/functions.hpp

            src/filter/FunctionFilter.cpp

            src/graph/ProtectionGraph.cpp

            src/trace/TraceableValue.cpp
            src/trace/PreservedValueRegistry.cpp

            src/util/functions.cpp

            src/AnalysisPass.cpp
            src/GraphPrinter.cpp
            src/GraphPass.cpp
            src/ProtectionPass.cpp
            src/MMap.h)
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
		/usr/include/llvm-3.9/
		/usr/include/llvm-c-3.9/
)

# LLVM is (typically) built with no C++ RTTI. We need to match that.
#target_compile_options(${COMPOSITION_FRAMEWORK_TARGET_NAME}
#                       PRIVATE
#                       -fno-rtti)

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
