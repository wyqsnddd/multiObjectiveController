cmake_minimum_required(VERSION 3.5)

message(STATUS "")
message(STATUS "============================================")
message(STATUS "             multi-Objective controller")
message(STATUS "============================================")
project(multiObjectiveController VERSION 1.0
	DESCRIPTION 	"A Multi-objective QP controller"
	LANGUAGES CXX
	)

include(cmake/base.cmake)
include(cmake/eigen.cmake)

# These variables have to be defined before running setup_project
set(PROJECT_URL https://github.com/wyqsnddd/multiObjectiveController)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_DISABLE_WERROR 1)
setup_project()
#enable_language(C)



search_for_eigen()


find_package(DART  REQUIRED COMPONENTS utils-urdf optimizer-nlopt gui-osg collision-bullet collision-ode CONFIG)
find_package(Bullet REQUIRED)
find_package(Boost  COMPONENTS program_options REQUIRED )

include_directories(${DART_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

file(GLOB lib_srcs  src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${lib_srcs})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME} 
 	PUBLIC	
	$<INSTALL_INTERFACE:include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} 
  	PUBLIC	
	${DART_LIBRARIES} 
	${NLOPT_LIBRARIES} 
	dart-optimizer-nlopt )
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

#===============================================================================
# Add sub-directories
#===============================================================================

add_subdirectory(examples)

setup_project_finalize()
