cmake_minimum_required(VERSION 2.8)
project(myslam)

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#SET(CMAKE_BUILD_TYPE DEBUG)
##IF(NOT CMAKE_BUILD_TYPE)
##  SET(CMAKE_BUILD_TYPE Release)
##ENDIF()
#
#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
#
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
##SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native ")
## Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
#else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set("OpenCV_DIR" "/usr/local/share/OpenCV/")
#find_package(OpenCV 2.4.3 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
MESSAGE("OpenCV vesion: ${OpenCV_VERSION}")
#find_package(Eigen3 3.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)
MESSAGE("Eigen3 vesion: ${Eigen3_VERSION}")
find_package(Pangolin REQUIRED)
find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
  REQUIRED             					# Fail with error if Boost is not found
  COMPONENTS serialization 				# Boost libraries by their canonical name
  )

include_directories(
../../   #${PROJECT_SOURCE_DIR}
../../include  #${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#add_library 生成了 libmyslam_s.so
add_library(${PROJECT_NAME} SHARED
../../src/System.cpp
../../src/Tracking.cpp
../../src/LocalMapping.cpp
../../src/LoopClosing.cpp
../../src/ORBextractor.cpp
../../src/ORBmatcher.cpp
../../src/FrameDrawer.cpp
../../src/Converter.cpp
../../src/MapPoint.cpp
../../src/KeyFrame.cpp
../../src/Map.cpp
../../src/MapDrawer.cpp
../../src/Optimizer.cpp
../../src/PnPsolver.cpp
../../src/Frame.cpp
../../src/KeyFrameDatabase.cpp
../../src/Sim3Solver.cpp
../../src/Initializer.cpp
../../src/Viewer.cpp
)
#set(LIBS
#-lboost_system
#)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
/home/ubuntu/catkin_ws/src/ORB_SLAM2_map_qt/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
/home/ubuntu/catkin_ws/src/ORB_SLAM2_map_qt/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
${Boost_LIBRARIES}
)

# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cpp)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cpp)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cpp)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cpp)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cpp)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cpp)
#target_link_libraries(mono_euroc ${PROJECT_NAME})


# sxp
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/myslam_run)
#生成调用摄像头可执行文件
#add_executable(myslam myslam.cpp)
#target_link_libraries(myslam ${PROJECT_NAME})

#生成调用myvideo.mp4 可执行文件
add_executable(myvideo myvideo.cpp)
target_link_libraries(myvideo ${PROJECT_NAME})

