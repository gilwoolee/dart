###############################################
# apps/handManipulation
set(APP_NAME handObjectRolling)
file(GLOB ${APP_NAME}_srcs "*.cpp")
file(GLOB ${APP_NAME}_hdrs "*.h")
add_executable(${APP_NAME} ${${APP_NAME}_srcs} ${${APP_NAME}_hdrs})
target_link_libraries(${APP_NAME} dart)
TARGET_LINK_LIBRARIES(${APP_NAME} snopt_c snprint_c blas_c f2c)
set_target_properties(${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

