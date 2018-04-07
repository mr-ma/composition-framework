PROJECT(constraints C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ConstraintHandlerPass MODULE
            ConstraintAnalysisPass.cpp
            ConflictGraph.cpp
            GraphPrinter.cpp
            ${INCLUDE_DIR_HEADERS})

set_target_properties(ConstraintHandlerPass PROPERTIES CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set_target_properties(ConstraintHandlerPass PROPERTIES CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")

target_link_libraries(ConstraintHandlerPass)
# LLVM is (typically) built with no C++ RTTI. We need to match that.
set_target_properties(ConstraintHandlerPass PROPERTIES COMPILE_FLAGS "-fno-rtti -g")
# Get proper shared-library behavior (where symbols are not necessarily
# resolved when the shared library is linked) on OS X.
#if(APPLE)
#    set_target_properties(ConstraintHandlerPass PROPERTIES
#        LINK_FLAGS "-undefined dynamic_lookup"
#    )
#endif(APPLE)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.58 COMPONENTS graph)

if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	target_link_libraries(ConstraintHandlerPass ${Boost_LIBRARIES})
endif()

install(DIRECTORY ./ DESTINATION /usr/local/include/constraints
        FILES_MATCHING PATTERN "*.h")
install(TARGETS ConstraintHandlerPass
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)