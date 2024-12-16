cmake_minimum_required(VERSION 3.5)

project(optoforce_sensor_redis_driver)

# CMAKE OPTIONS
set(CMAKE_CXX_FLAGS "-std=c++17")
# set default build to release with debug info. mimic'ed from RBDL project.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  # Set the default build, and also the possible values of build type for
  # cmake-gui
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

# optoforce
set(OPTOFORCE_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/OptoForceAPI2.0.0/include/)
find_library(OMD_LIBRARIES NAMES OptoForceAPI PATHS ${CMAKE_CURRENT_LIST_DIR}/OptoForceAPI2.0.0/lib)
include_directories(${OPTOFORCE_INCLUDE_DIRS})

# - eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# BUILD SOURCES AND LIBRARY
set(SOURCES optoforce6DRedisDriver.cpp )

# Create an executable
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OMD_LIBRARIES} -lncurses -lrt -lpthread)



