###############################################
# apps/testImpulseDynamics
file(GLOB testImpulseDynamics_srcs "*.cpp")
file(GLOB testImpulseDynamics_hdrs "*.h")
add_executable(testImpulseDynamics ${testImpulseDynamics_srcs} ${testImpulseDynamics_hdrs})
target_link_libraries(testImpulseDynamics dart)
set_target_properties(testImpulseDynamics PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
