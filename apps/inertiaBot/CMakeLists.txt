###############################################
# apps/cubes
file(GLOB inertiaBot_srcs "*.cpp")
file(GLOB inertiaBot_hdrs "*.h")
add_executable(inertiaBot ${inertiaBot_srcs} ${inertiaBot_hdrs})
target_link_libraries(inertiaBot dart nlopt)
set_target_properties(inertiaBot PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
