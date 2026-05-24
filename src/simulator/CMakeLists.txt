set(LIB_NAME simulator)

# Collect source and headers
file(GLOB_RECURSE SIMULATOR_LIB_SOURCES_HEADERS CONFIGURE_DEPENDS 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# Create library
add_library(${LIB_NAME} ${SIMULATOR_LIB_SOURCES_HEADERS})


# Expose library as simulator/*.h
target_include_directories(${LIB_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..   
)

# Link library dependenceis
target_link_libraries(${LIB_NAME}
    PUBLIC
        eigen
        ode
)
