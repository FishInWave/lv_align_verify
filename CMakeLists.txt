cmake_minimum_required(VERSION 2.8.3)
project(lv_align_verify)
add_compile_options(-std=c++14)
add_definitions(-Wno-deprecated-declarations)
SET(CMAKE_BUILD_TYPE Release)


LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")
# catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
)
generate_dynamic_reconfigure_options(
  config/LVAlignVerify.cfg
)
# PCL
find_package(PCL REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS} )
message("PCL [OK]: " ${PCL_INCLUDE_DIRS})

# ZED SDK3
find_package(ZED 3 REQUIRED QUIET)
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
message("ZED SDK [OK]: " ${ZED_INCLUDE_DIRS} )

# OPENCV
find_package(OpenCV 3.4 REQUIRED QUIET)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
message("OpenCV [OK]: " ${OpenCV_INCLUDE_DIRS} )

# CUDA
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
message("CUDA [OK]: " ${CUDA_INCLUDE_DIRS} )


#GLOG (neccessary)
FIND_PACKAGE(GLOG REQUIRED QUIET)
IF(GLOG_FOUND)
INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIRS})
MESSAGE("GLOG [OK]:" ${GLOG_INCLUDE_DIRS}) 
ENDIF(GLOG_FOUND)

# OpenMP
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Ceres
FIND_PACKAGE(Ceres REQUIRED QUIET)
IF(Ceres_FOUND)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("CERES [OK]:" ${CERES_INCLUDE_DIRS}) 
ADD_DEFINITIONS(-DCERES_ON)
ENDIF(Ceres_FOUND)
add_message_files(
  DIRECTORY msg
  FILES
  Point2DPolarMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL OpenCV ZED
  LIBRARIES ros_msg
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
# list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${catkin_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${GLOG_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${CERES_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${ZED_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${CUDA_CUDA_LIBRARY})
SET(DEP_LIBS ${DEP_LIBS} ${CUDA_LIBRARY})
SET(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_node ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}_node
  ${DEP_LIBS}
)
target_link_libraries(${PROJECT_NAME}_node libyaml-cpp.so) # yaml需要so
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp 
  ${PROJECT_NAME}_gencfg)





