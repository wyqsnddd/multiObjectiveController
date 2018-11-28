cmake_minimum_required(VERSION 2.8.12)

# set(CMAKE_PREFIX_PATH "/home/yuquan/local/tiny-dnn/build")


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CONFIG_INSTALL_DIR "share/${PROJECT_NAME}/cmake")
set(INCLUDE_INSTALL_DIR "include")
set(LIBRARY_INSTALL_DIR "lib")

project(impactControllers)

#find_package(OSG)

get_property(PACKAGE_INCLUDE_DIRS GLOBAL
  PROPERTY "${PROJECT_NAME_UPPERCASE}_INCLUDE_DIRS")

# Set relative location to install additional documentation (sample data,
# examples, and tutorials)

# Set relative location to install additional documentation (sample data,
# examples, and tutorials)
set(DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH "share/doc/${PROJECT_NAME}")

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")


include(DARTMacros)

# CMake component helper. It uses the following variables:
# - LIBRARY_INSTALL_DIR
# - CONFIG_INSTALL_DIR
include(Components)
initialize_component_helpers(${PROJECT_NAME})
#===============================================================================
# Project settings
#===============================================================================

# If you change the version, please update the <version> tag in package.xml.
set(DART_MAJOR_VERSION "7")
set(DART_MINOR_VERSION "0")
set(DART_PATCH_VERSION "0")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(DART_PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_PKG_EXTERNAL_DEPS "eigen, ccd, fcl, assimp, boost")

#===============================================================================
# Build options
#===============================================================================
# cmake -DDART_VERBOSE=ON ..
option(DART_VERBOSE "Whether print detailed information in CMake process" OFF)
option(ENABLE_OPENMP "Build with OpenMP parallaization enabled" ON)
if(MSVC)
  set(DART_RUNTIME_LIBRARY "/MD" CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE DART_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  option(DART_MSVC_DEFAULT_OPTIONS "Build DART with default Visual Studio options" OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()
# Warning: DART_ENABLE_SIMD should be ON only when you build DART and the DART
# dependent projects on the same machine. If this option is on, then compile
# option `-march=native` is added to the target `dart` that enables all
# instruction subsets supported by the local machine. If the architecture of
# local machines are different then the projects will be built with different
# compile options, which may cause runtime errors especially memory alignment
# errors.
option(DART_ENABLE_SIMD
  "Build DART with all SIMD instructions on the current local machine" OFF)
option(DART_BUILD_GUI_OSG "Build osgDart library" ON)
option(DART_BUILD_EXTRAS "Build extra projects" OFF)
option(DART_CODECOV "Turn on codecov support" OFF)
option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(DART_FAST_DEBUG "Add -O1 option for DEBUG mode build" OFF)

#===============================================================================
# Print intro
#===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                DART ${DART_VERSION}")
message(STATUS "============================================")

#===============================================================================
# CodeCov settings
#===============================================================================
if(DART_CODECOV)
  include(CodeCoverage)
  setup_target_for_coverage(codecov ctest coverage)
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage"
  )
endif()

#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG FALSE)
set(BUILD_TYPE_RELEASE FALSE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
  set(BUILD_TYPE_MINSIZEREL TRUE)
else()
  message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Find dependencies
#===============================================================================
include(DARTFindDependencies)

#===============================================================================
# Include directories
#===============================================================================
include_directories(BEFORE ${PROJECT_SOURCE_DIR})
include_directories("${CMAKE_BINARY_DIR}")


#===============================================================================
# CMake configuration files for components and targets
#===============================================================================
# Generate and install CMake configuration files for each component <C>:
# - <C>Component.cmake, which defines:
#   - dart_<C>_DEPENDENCIES: list of component dependencies
#   - dart_<C>_LIBRARIES: list of library targets in this component
# - <C>Targets.cmake, which creates IMPORTED targets
install_component_exports(${PROJECT_NAME})


# Generate the DART CMake Config and version files
include(CMakePackageConfigHelpers)
set(DART_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME_UPPERCASE}Config.cmake.in)
set(DART_CONFIG_OUT ${CMAKE_BINARY_DIR}/${PROJECT_NAME_UPPERCASE}Config.cmake)
set(DART_VERSION_OUT ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME_UPPERCASE}ConfigVersion.cmake)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_OUT})
  message(STATUS ${DART_VERSION_OUT})
endif()
# configure_package_config_file(
#   ${DART_CONFIG_IN}
#   ${DART_CONFIG_OUT}
#   INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
#   PATH_VARS INCLUDE_INSTALL_DIR
#   )
# write_basic_config_version_file(
#   ${DART_VERSION_OUT}
#   VERSION ${${PROJECT_NAME_UPPERCASE}_VERSION}
#   COMPATIBILITY SameMajorVersion
#   )
# install(
#   FILES ${DART_CONFIG_OUT} ${DART_VERSION_OUT}
#   DESTINATION "${CONFIG_INSTALL_DIR}")

# Generate the DART pkg-config
set(PC_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/dart.pc)
if(DART_VERBOSE)
  message(STATUS ${PC_CONFIG_OUT})
endif()
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig)

#===============================================================================
# Print build summary
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Build summary ]")
  message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
  message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler version : ${CXX_COMPILER_VERSION}")
  message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
  message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
  message(STATUS "ENABLE_OPENMP    : ${ENABLE_OPENMP}")
  message(STATUS "Build gui::osg   : ${DART_BUILD_GUI_OSG}")
  message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
  message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
  if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
    message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
    message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
    message(STATUS "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
    message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
  endif()
  message(STATUS "CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
  message(STATUS "CMAKE_BINARY_DIR : ${CMAKE_BINARY_DIR}")
endif(DART_VERBOSE)
#===============================================================================
# Add control libraries 
#===============================================================================
add_compile_options(-std=c++11)
# find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui-osg dart-opimizer CONFIG)

file(GLOB lib_srcs "include/*" "src/*.cpp")
find_package(NLOPT)
#set(lib_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/"*.cpp")
include_directories(${DART_INCLUDE_DIRS})
include_directories(${NLOPT_INCLUDE_DIRS})
include_directories("include")
add_library(${PROJECT_NAME} ${lib_srcs})
set_target_properties(${PROJECT_NAME}  PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} ${NLOPT_LIBRARIES} dart-optimizer-nlopt ) 
#===============================================================================
# Add sub-directories
#===============================================================================

#add_subdirectory(manipulatorExampleOne)
add_subdirectory(impactExampleOne)
#add_subdirectory(osgBoxStacking)
