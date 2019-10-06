link_directories(${CMAKE_SOURCE_DIR}/lib)

#file(GLOB srcs "*.cpp" "*.hpp")

file(GLOB impact_srcs impactExampleOne.cpp  "*.hpp")

add_executable(impactExampleOne ${impact_srcs})
target_link_libraries(impactExampleOne 
  	PUBLIC	
	${DART_LIBRARIES} ${NLOPT_LIBRARIES} ${PROJECT_NAME})
set_target_properties(impactExampleOne PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")



file(GLOB collision_srcs collisionExampleOne.cpp  "*.hpp")

add_executable(collisionExampleOne ${collision_srcs})
target_link_libraries(collisionExampleOne 
 	PUBLIC	
	${DART_LIBRARIES} ${NLOPT_LIBRARIES} ${PROJECT_NAME})
set_target_properties(collisionExampleOne PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

