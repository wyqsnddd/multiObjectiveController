link_directories(${CMAKE_SOURCE_DIR}/lib)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(impactExampleOne ${srcs})

target_link_libraries(impactExampleOne ${DART_LIBRARIES} ${NLOPT_LIBRARIES} impactControllers)
set_target_properties(impactExampleOne PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

