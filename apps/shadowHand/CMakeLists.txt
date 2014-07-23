###############################################
# apps/shadowHand
file(GLOB shadowHand_srcs "*.cpp")
file(GLOB shadowHand_hdrs "*.h")
add_executable(shadowHand ${shadowHand_srcs} ${shadowHand_hdrs})
target_link_libraries(shadowHand dart)
set_target_properties(shadowHand PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
