cmake_minimum_required(VERSION 2.8)

message(STATUS "")
message(STATUS "============================================")
message(STATUS "             multi-Objective controller")
message(STATUS "============================================")


include(cmake/base.cmake)
include(cmake/eigen.cmake)

# These variables have to be defined before running setup_project
set(PROJECT_NAME impactControllers)
set(PROJECT_DESCRIPTION "A Multi-objective QP controller")
set(PROJECT_URL https://github.com/wyqsnddd/multiObjectiveController)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_DISABLE_WERROR 1)
setup_project()
enable_language(C)



search_for_eigen()


find_package(DART  REQUIRED COMPONENTS utils-urdf optimizer-nlopt gui-osg collision-bullet CONFIG)
find_package(Bullet REQUIRED)
FIND_PACKAGE(Boost  COMPONENTS program_options REQUIRED )

include_directories(${DART_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS})



file(GLOB lib_srcs  src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${lib_srcs})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} ${NLOPT_LIBRARIES} dart-optimizer-nlopt ) 
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

#===============================================================================
# Add sub-directories
#===============================================================================

add_subdirectory(examples)

setup_project_finalize()
