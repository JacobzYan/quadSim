set(LIB_NAME simulator_sandbox)

# Collect source and headers
file(GLOB_RECURSE SANDBOX_SOURCES_HEADERS CONFIGURE_DEPENDS 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# Create library
add_library(${LIB_NAME} ${SANDBOX_SOURCES_HEADERS})


# Expose library as simulator_sandbox/*.h
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



# Define exe name
set (ODETESTTARGET odeTest)

# ==== Helper Files ====
file(GLOB_RECURSE SANDBOX_SOURCES_HEADERS CONFIGURE_DEPENDS 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# add_subdirectory(odeFuncLib)
# Link to executable
add_executable(${ODETESTTARGET} ${SANDBOX_SOURCES_HEADERS})


# Link dependencies
target_link_libraries(${ODETESTTARGET}
    PUBLIC
        eigen
        ode
)

# Set output to bin
set_target_properties(${ODETESTTARGET}
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/testers"
)