PROJECT(constraint-handler C CXX)

add_library(ConstraintHandlerPass MODULE
            ConstraintAnalysisPass.cpp
            ConflictGraph.cpp ConflictGraph.h)

set_target_properties(ConstraintHandlerPass PROPERTIES CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set_target_properties(ConstraintHandlerPass PROPERTIES CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")


if ($ENV{CLION_IDE})
	include_directories("/usr/include/llvm-3.9/")
	include_directories("/usr/include/llvm-c-3.9/")
endif ()

add_subdirectory(third_party/stlplus)
target_link_libraries(ConstraintHandlerPass stlplus)
target_include_directories(ConstraintHandlerPass PUBLIC third_party/)
# LLVM is (typically) built with no C++ RTTI. We need to match that.
set_target_properties(ConstraintHandlerPass PROPERTIES COMPILE_FLAGS "-fno-rtti -g")
# Get proper shared-library behavior (where symbols are not necessarily
# resolved when the shared library is linked) on OS X.
#if(APPLE)
#    set_target_properties(ConstraintHandlerPass PROPERTIES
#        LINK_FLAGS "-undefined dynamic_lookup"
#    )
#endif(APPLE)
