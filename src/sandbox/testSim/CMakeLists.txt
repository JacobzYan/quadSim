set (SIMTARGET simTest)

# ==== Helper Files ====
file(GLOB_RECURSE SOURCES_HEADERS CONFIGURE_DEPENDS 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# add_subdirectory(odeFuncLib)
# Link to executable
add_executable(${SIMTARGET} ${SOURCES_HEADERS})


# Link dependencies
# Link dependencies
target_link_libraries(${SIMTARGET}
    PRIVATE
        simulator
)

# target_link_libraries(${SIMTARGET}
#     PUBLIC
#         eigen
#         ode
#     PRIVATE
#         simulator
# )

# Set output to bin
set_target_properties(${SIMTARGET}
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/testers/testSim"
)   