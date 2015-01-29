###############################################
# apps/multipleShapes
file(GLOB multipleShapes_srcs "*.cpp")
file(GLOB multipleShapes_hdrs "*.h")
add_executable(multipleShapes ${multipleShapes_srcs} ${multipleShapes_hdrs})
target_link_libraries(multipleShapes dart)
set_target_properties(multipleShapes PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
