###############################################
# apps/balance
file(GLOB yunfei_srcs "*.cpp")
file(GLOB yunfei_hdrs "*.h")
add_executable(yunfei ${yunfei_srcs} ${yunfei_hdrs})
target_link_libraries(yunfei dart)
set_target_properties(yunfei PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
