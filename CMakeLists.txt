cmake_minimum_required(VERSION 3.8)
project(constraint-handler VERSION 0.1 LANGUAGES CXX)


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
set(CONSTRAINT_HANDLER_TARGET_NAME ${PROJECT_NAME})
set(CONSTRAINT_HANDLER_CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}"
    CACHE INTERNAL "")
set(CONSTRAINT_HANDLER_INCLUDE_INSTALL_DIR "include")
set(CONSTRAINT_HANDLER_TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(CONSTRAINT_HANDLER_CMAKE_CONFIG_TEMPLATE "cmake/config.cmake.in")
set(CONSTRAINT_HANDLER_CMAKE_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(CONSTRAINT_HANDLER_CMAKE_VERSION_CONFIG_FILE "${CONSTRAINT_HANDLER_CMAKE_CONFIG_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(CONSTRAINT_HANDLER_CMAKE_PROJECT_CONFIG_FILE "${CONSTRAINT_HANDLER_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Config.cmake")
set(CONSTRAINT_HANDLER_CMAKE_PROJECT_TARGETS_FILE "${CONSTRAINT_HANDLER_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Targets.cmake")
set(CONSTRAINT_HANDLER_INCLUDE_BUILD_DIR "${PROJECT_SOURCE_DIR}/include/")


##
## Dependencies
##
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.58 REQUIRED graph)
find_package(LLVM REQUIRED CONFIG)
find_package(composition-framework REQUIRED)
##
## TARGET
## create target and add include path
##

add_library(${CONSTRAINT_HANDLER_TARGET_NAME} MODULE
            include/constraints/AnalysisPass.hpp
            include/constraints/ConflictGraph.hpp
            include/constraints/GraphPrinter.hpp
            include/constraints/GraphPass.hpp
            include/constraints/ProtectionPass.hpp
            src/AnalysisPass.cpp
            src/ConflictGraph.cpp
            src/GraphPrinter.cpp
            src/GraphPass.cpp
            src/ProtectionPass.cpp
            src/MMap.h)
add_library(${PROJECT_NAME}::${CONSTRAINT_HANDLER_TARGET_NAME} ALIAS ${CONSTRAINT_HANDLER_TARGET_NAME})
target_compile_features(${CONSTRAINT_HANDLER_TARGET_NAME} PUBLIC cxx_std_14)


target_include_directories(
		${CONSTRAINT_HANDLER_TARGET_NAME}
		PUBLIC
		$<BUILD_INTERFACE:${CONSTRAINT_HANDLER_INCLUDE_BUILD_DIR}>
		$<INSTALL_INTERFACE:include>
		PRIVATE
		${Boost_INCLUDE_DIRS}
		${LLVM_INCLUDE_DIRS}
		${composition-framework_INCLUDE_DIR}
		/usr/include/llvm-3.9/
		/usr/include/llvm-c-3.9/
)

# LLVM is (typically) built with no C++ RTTI. We need to match that.
target_compile_options(${CONSTRAINT_HANDLER_TARGET_NAME}
                       PRIVATE
                       -fno-rtti)

target_link_libraries(${CONSTRAINT_HANDLER_TARGET_NAME}
                      PRIVATE
                      ${Boost_LIBRARIES})
##
## INSTALL
## install header files, generate and install cmake config files for find_package()
##
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
		${CONSTRAINT_HANDLER_CMAKE_VERSION_CONFIG_FILE} COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
		${CONSTRAINT_HANDLER_CMAKE_CONFIG_TEMPLATE}
		${CONSTRAINT_HANDLER_CMAKE_PROJECT_CONFIG_FILE}
		INSTALL_DESTINATION ${CONSTRAINT_HANDLER_CONFIG_INSTALL_DIR}
)

install(
		DIRECTORY ${CONSTRAINT_HANDLER_INCLUDE_BUILD_DIR}
		DESTINATION ${CONSTRAINT_HANDLER_INCLUDE_INSTALL_DIR}
)
install(
		FILES ${CONSTRAINT_HANDLER_CMAKE_PROJECT_CONFIG_FILE} ${CONSTRAINT_HANDLER_CMAKE_VERSION_CONFIG_FILE}
		DESTINATION ${CONSTRAINT_HANDLER_CONFIG_INSTALL_DIR}
)

export(
		TARGETS ${CONSTRAINT_HANDLER_TARGET_NAME}
		NAMESPACE ${PROJECT_NAME}::
		FILE ${CONSTRAINT_HANDLER_CMAKE_PROJECT_TARGETS_FILE}
)
install(
		TARGETS ${CONSTRAINT_HANDLER_TARGET_NAME}
		EXPORT ${CONSTRAINT_HANDLER_TARGETS_EXPORT_NAME}
		INCLUDES DESTINATION ${CONSTRAINT_HANDLER_INCLUDE_INSTALL_DIR}
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
)
install(
		EXPORT ${CONSTRAINT_HANDLER_TARGETS_EXPORT_NAME}
		NAMESPACE ${PROJECT_NAME}::
		DESTINATION ${CONSTRAINT_HANDLER_CONFIG_INSTALL_DIR}
)
