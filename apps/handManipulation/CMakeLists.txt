###############################################
# apps/manipulator
PROJECT(manipulator)
FILE(GLOB manipulator_srcs "*.cpp")
FILE(GLOB manipulator_hdrs "*.h")
ADD_EXECUTABLE(manipulator ${manipulator_srcs} ${manipulator_hdrs})
TARGET_LINK_LIBRARIES(manipulator dart ${DARTExt_LIBRARIES})
TARGET_LINK_LIBRARIES(manipulator snopt_c snprint_c blas_c f2c)
SET_TARGET_PROPERTIES(manipulator PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
