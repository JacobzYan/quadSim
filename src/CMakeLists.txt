include(FetchContent) # Fetch content to pull dependencies at configure time

# exe main file
set(TARGET testExe)




## Helper files

# Collect Helper Sources
# Adds all *.cc files in these subdirectories - technically bad practice but saves needing to add every file manually
# file(GLOB_RECURSE SRC_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")
# Also consider using list(FILTER <Variable name> EXCLUDE REGEX "<regex here>")
file(GLOB_RECURSE ALL_SOURCES_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/run/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/run/*.h"
    )



# Create Executables - Consider moving this to a subdirectory
add_executable (${TARGET} ${ALL_SOURCES_HEADERS})







# Dependencies
# location of .dll or .lib files
# target_link_libraries(${TARGET}
#     PRIVATE
#         ode
#         eigen
# )

# Set up dependencies
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        5.0.0
)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(
    libode
    GIT_REPOSITORY https://github.com/markmbaum/libode.git
    GIT_TAG        2d8ac5b
)
FetchContent_MakeAvailable(libode)
# Libode is not a cmake project by default - set it up
# file(GLOB_RECURSE LIBODE_SOURCES "${libode_SOURCE_DIR}/src/*.cc") # Build libode ourselves
# file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/helperDirectories") # Make helper directories
# if(NOT EXISTS "${CMAKE_BINARY_DIR}/helperDirectories/ode") # Alias libode header folder so it can be called as ode/<foo>.h
#     file(CREATE_LINK
#          "${libode_SOURCE_DIR}/src"
#          "${CMAKE_BINARY_DIR}/helperDirectories/ode"
#          SYMBOLIC)
# endif()
# add_library(ode STATIC ${LIBODE_SOURCES})
# target_include_directories(ode PUBLIC "${CMAKE_BINARY_DIR}/helperDirectories") # expose helperdirectory libraries
# Fix libode's include path — it exposes include/ode/ but we need include/
# so that <ode/ode_dopri_54.h> resolves correctly
target_include_directories(ode PUBLIC 
    $<BUILD_INTERFACE:${libode_SOURCE_DIR}/include>
)


target_link_libraries(${TARGET}
    PRIVATE
        ode
        eigen
)




# Add CMakeLists.txt files in subdirectories
add_subdirectory(simulator) # Files related to simulation of quad
add_subdirectory(run) # Top level execution scripts
add_subdirectory(sandbox) # Testers and experiments



# My Libraries
target_link_libraries(${TARGET}
    PRIVATE
    simulator
)


# Set output to bin
set_target_properties(${TARGET}
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)