set (CONTROLTARGET controlTest)

# ==== Helper Files ====
file(GLOB_RECURSE SOURCES_HEADERS CONFIGURE_DEPENDS 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# add_subdirectory(odeFuncLib)
# Link to executable
add_executable(${CONTROLTARGET} ${SOURCES_HEADERS})


# Link dependencies
target_link_libraries(${CONTROLTARGET}
    PRIVATE
        simulator
)

# Set output to bin
set_target_properties(${CONTROLTARGET}
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/testers/testControlSim"
)   