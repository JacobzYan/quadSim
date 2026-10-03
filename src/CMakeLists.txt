include(FetchContent) # Fetch content to pull dependencies at configure time


# Collect Helper Sources
# Adds all *.cc files in these subdirectories - technically bad practice but saves needing to add every file manually
# file(GLOB_RECURSE SRC_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")
# Also consider using list(FILTER <Variable name> EXCLUDE REGEX "<regex here>")
file(GLOB_RECURSE ALL_SOURCES_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/run/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/run/*.h"
    )


# External dependencies - location of .dll or .lib files
FetchContent_Declare( # Setup eigen
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        5.0.0
)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare( # Setup libode
    libode
    GIT_REPOSITORY https://github.com/markmbaum/libode.git
    GIT_TAG        2d8ac5b
)
FetchContent_MakeAvailable(libode)
target_include_directories(ode PUBLIC 
    $<BUILD_INTERFACE:${libode_SOURCE_DIR}/include>
)


# Add CMakeLists.txt files in subdirectories
add_subdirectory(simulator) # Files related to simulation of quad
add_subdirectory(run) # Top level execution scripts
add_subdirectory(sandbox) # Testers and experiments

