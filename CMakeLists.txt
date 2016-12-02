# cmake requirements
cmake_minimum_required(VERSION 2.8)

option(BUILD_ROS "Build with ROS" OFF)
mark_as_advanced(BUILD_ROS)

if(BUILD_ROS)
        include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
	rosbuild_init()
	add_definitions(-DBUILD_WITH_ROS)
	SET(ROS_BUILD_TYPE Release)
endif(BUILD_ROS)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# PUTSLAM Project configuration
PROJECT(PUTSLAM)
SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -std=c++11 -pthread -fopenmp")

if(BUILD_ROS)
	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		geometry_msgs
		message_filters
		roscpp
		rospy
		sensor_msgs
	)
endif(BUILD_ROS)

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/putslam")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/putslam/Defs")

# core libraries
option(BUILD_PUTSLAM "Build PUT SLAM libraries" ON)
option(BUILD_PUTSLAM_DEFS "Build PUTSLAM Defs library" ON)
mark_as_advanced(BUILD_PUTSLAM_DEFS)
option(BUILD_PUTSLAM_UTILITIES "Build PUTSLAM Utilities library" ON)
mark_as_advanced(BUILD_PUTSLAM_UTILITIES)
option(BUILD_PUTSLAM_GRABBER "Build PUTSLAM Grabber library" ON)
mark_as_advanced(BUILD_PUTSLAM_GRABBER)
option(BUILD_PUTSLAM_GRABBER_PTGREY "Build PUTSLAM Grabber PointGrey library" OFF)
mark_as_advanced(BUILD_PUTSLAM_GRABBER_PTGREY)
option(BUILD_PUTSLAM_TRACKER "Build PUTSLAM Tracker library" ON)
mark_as_advanced(BUILD_PUTSLAM_TRACKER)
option(BUILD_PUTSLAM_LDB "Build PUTSLAM LDB library" ON)
mark_as_advanced(BUILD_PUTSLAM_LDB)
option(BUILD_PUTSLAM_RGBD "Build PUTSLAM RGBD library" ON)
mark_as_advanced(BUILD_PUTSLAM_RGBD)
option(BUILD_PUTSLAM_MATCHER "Build PUTSLAM Matcher library" ON)
mark_as_advanced(BUILD_PUTSLAM_MATCHER)
option(BUILD_PUTSLAM_MOTIONMODEL "Build PUTSLAM MotionModel library" ON)
mark_as_advanced(BUILD_PUTSLAM_MOTIONMODEL)
option(BUILD_PUTSLAM_TRANSFORM_ESTIMATOR "Build PUTSLAM Transformation Estimation library" ON)
mark_as_advanced(BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)
#option(BUILD_PUTSLAM_USAC "Build PUTSLAM USAC library" ON)
#mark_as_advanced(BUILD_PUTSLAM_USAC)
option(BUILD_PUTSLAM_POSE_GRAPH "Build PUTSLAM Pose Graph (local and global graph) library" ON)
mark_as_advanced(BUILD_PUTSLAM_POSE_GRAPH)
option(BUILD_PUTSLAM_MAP "Build PUTSLAM Map library" ON)
mark_as_advanced(BUILD_PUTSLAM_MAP)
option(BUILD_PUTSLAM_LOOPCLOSURE "Build PUTSLAM loop closure library" ON)
mark_as_advanced(BUILD_PUTSLAM_LOOPCLOSURE)
option(BUILD_PUTSLAM_VISUALPLACERECOGNITION "Build PUTSLAM visual place recognition" ON)
mark_as_advanced(BUILD_PUTSLAM_VISUALPLACERECOGNITION)
option(BUILD_PUTSLAM_PATCH "Build PUTSLAM Patches" ON)
mark_as_advanced(BUILD_PUTSLAM_PATCH)
option(BUILD_PUTSLAM_VISUALIZER "Build PUTSLAM Visualizer" ON)
mark_as_advanced(BUILD_PUTSLAM_VISUALIZER)
option(BUILD_PUTSLAM_DEMO_PLAYER "Build g2o player demo" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_PLAYER)

# demos
option(BUILD_PUTSLAM_DEMO "Build PUT SLAM demonstration programs and libraries" ON)
option(BUILD_PUTSLAM_DEMO_G2O "Build g2o PUT SLAM test" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_G2O)
option(BUILD_PUTSLAM_DEMO_BASIC "Build basic PUT SLAM test" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_BASIC)
option(BUILD_PUTSLAM_DEMO_GRABBER "Build demo for grabber" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_GRABBER)
option(BUILD_PUTSLAM_DEMO_KABSCH "Build demo for Kabsch" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_KABSCH)
option(BUILD_PUTSLAM_DEMO_MAP "Build demo for features map" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_MAP)
option(BUILD_PUTSLAM_GRAPHCONVERTER "Build g2o 2D graph to 3D graph converter" ON)
mark_as_advanced(BUILD_PUTSLAM_GRAPHCONVERTER)
option(BUILD_PUTSLAM_DEMO_VISUALIZER "Build g2o visualizer demo" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_VISUALIZER)
option(BUILD_PUTSLAM_DEMO_ROS "Build ROS demo" ON)
mark_as_advanced(BUILD_PUTSLAM_DEMO_ROS)
#additional dependencies

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# PCL
#FIND_PACKAGE(PCL REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#octomap library
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#g2o library
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
link_directories(${G2O_LIBRARY_DIRS})

#CSparse library
FIND_PACKAGE(CSparse)
include_directories(${LIBCSPARSE_INCLUDE_DIR})
link_directories(${LIBCSPARSE_LIBRARY})

#Eigen
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rdParty/Eigen)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#QGLViewer library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
FIND_PACKAGE(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OPENGL_LIBRARY})

#QGLViewer library
FIND_PACKAGE(QGLViewer)
include_directories(${QGLVIEWER_INCLUDE_DIR})
include_directories(${QT_INCLUDES})
link_directories(${QGLVIEWER_LIBRARY})

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

if (MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

if(BUILD_ROS)
	include_directories(
	  ${catkin_INCLUDE_DIRS}
	)
endif(BUILD_ROS)

###############################################################################
#
# PUTSLAM tinyXML2 library
#
###############################################################################

if(BUILD_ROS)
	rosbuild_add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)
else()
	add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)
endif(BUILD_ROS)

###############################################################################
#
# PUTSLAM defs library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_DEFS)
        file(GLOB DEFS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Defs/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_DEFS STATIC ${DEFS_HEADERS})
        else()
                add_library(Putslam_DEFS STATIC ${DEFS_HEADERS})
        endif(BUILD_ROS)
        set_target_properties(Putslam_DEFS PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS PUTSLAMDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/putslam/Defs/)

endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_DEFS)

###############################################################################
#
# PUTSLAM Utilities library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_UTILITIES)
        file(GLOB Putslam_UTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB Putslam_UTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Utilities/*.h"
        )

        if(BUILD_ROS)
                rosbuild_add_library(Putslam_Utilities STATIC ${Putslam_UTILITIES_SOURCES} ${Putslam_UTILITIES_HEADERS})
        else()
                add_library(Putslam_Utilities STATIC ${Putslam_UTILITIES_SOURCES} ${Putslam_TILITIES_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS Putslam_Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${TRACKER_HEADERS} DESTINATION include/putslam/Utilities/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_UTILITIES)

###############################################################################
#
# PUTSLAM Grabber library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)
        INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/Include")
        if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            add_definitions(-DWITH_PTGREY)
            INCLUDE_DIRECTORIES("/usr/include/flycapture")
        endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
        LINK_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/Lib")
        file(GLOB Putslam_GRABBER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Grabber/calibration.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/depthSensorModel.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/kinectGrabber.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/fileGrabber.cpp"
            #if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
                "${CMAKE_SOURCE_DIR}/src/Grabber/ptgreyGrabber.cpp"
            #endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            "${CMAKE_SOURCE_DIR}/src/Grabber/templateGrabber.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/xtionGrabber.cpp"
            if(BUILD_ROS)
                 "${CMAKE_SOURCE_DIR}/src/Grabber/ROSGrabber.cpp"
            endif(BUILD_ROS)
        )
        file(GLOB Putslam_GRABBER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/grabber.h"
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/calibration.h"
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/depthSensorModel.h"
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/kinectGrabber.h"
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/fileGrabber.h"
            #if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
                "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/ptgreyGrabber.h"
            #endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/templateGrabber.h"
            "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/xtionGrabber.h"
            if(BUILD_ROS)
                    "${CMAKE_SOURCE_DIR}/include/putslam/Grabber/ROSGrabber.h"
            endif(BUILD_ROS)
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_Grabber STATIC ${Putslam_GRABBER_SOURCES} ${Putslam_GRABBER_HEADERS})
                TARGET_LINK_LIBRARIES(Putslam_Grabber OpenNI2 ${OpenCV_LIBS} ${catkin_LIBRARIES})
        else()
                add_library(Putslam_Grabber STATIC ${Putslam_GRABBER_SOURCES} ${Putslam_GRABBER_HEADERS})
                TARGET_LINK_LIBRARIES(Putslam_Grabber OpenNI2 ${OpenCV_LIBS})
        endif(BUILD_ROS)
        if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            TARGET_LINK_LIBRARIES(Putslam_Grabber OpenNI2 ${OpenCV_LIBS} flycapture-c flycapture)
        endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
        file(INSTALL "${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/Lib/" DESTINATION bin)
        INSTALL(TARGETS Putslam_Grabber RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${GRABBER_HEADERS} GRABBERKINECT_HEADERS DESTINATION include/putslam/Grabber/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)

###############################################################################
#
# PUTSLAM Tracker library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRACKER)
        file(GLOB Putslam_TRACKER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Tracker/*.cpp"
        )
        file(GLOB Putslam_TRACKER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Tracker/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_Tracker STATIC ${Putslam_TRACKER_SOURCES} ${Putslam_TRACKER_HEADERS})
        else()
                add_library(Putslam_Tracker STATIC ${Putslam_TRACKER_SOURCES} ${Putslam_TRACKER_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS Putslam_Tracker RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_TRACKER_HEADERS} DESTINATION include/putslam/Tracker/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRACKER)

###############################################################################
#
# PUTSLAM Transformation Estimation library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)
        file(GLOB Putslam_TRANSFORM_ESTIMATOR_SOURCES
            "${CMAKE_SOURCE_DIR}/src/TransformEst/*.cpp"
        )
        file(GLOB Putslam_TRANSFORM_ESTIMATOR_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/TransformEst/*.h"
        )

        if(BUILD_ROS)
                rosbuild_add_library(Putslam_TransformEst STATIC ${Putslam_TRANSFORM_ESTIMATOR_SOURCES} ${Putslam_TRANSFORM_ESTIMATOR_HEADERS})
        else()
                add_library(Putslam_TransformEst STATIC ${Putslam_TRANSFORM_ESTIMATOR_SOURCES} ${Putslam_TRANSFORM_ESTIMATOR_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_TransformEst Putslam_RGBD)
        INSTALL(TARGETS Putslam_TransformEst RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_TRANSFORM_ESTIMATOR_HEADERS} DESTINATION include/putslam/TransformEst/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)

###############################################################################
#
# PUTSLAM USAC library
#
###############################################################################

#if(BUILD_PUTSLAM AND BUILD_PUTSLAM_USAC)
#        file(GLOB Putslam_USAC_SOURCES
#            "${CMAKE_SOURCE_DIR}/src/USAC/*.cpp"
#        )
#        file(GLOB Putslam_USAC_HEADERS
#            "${CMAKE_SOURCE_DIR}/include/putslam/USAC/*.h"
#        )
#
#        ADD_LIBRARY(Putslam_USAC STATIC ${Putslam_USAC_SOURCES} ${Putslam_USAC_HEADERS})
#        TARGET_LINK_LIBRARIES(Putslam_USAC Putslam_RGBD)
#        INSTALL(TARGETS Putslam_USAC RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${Putslam_USAC_HEADERS} DESTINATION include/putslam/USAC/)
#endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_USAC)

###############################################################################
#
# PUTSLAM LDB library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_LDB)
        file(GLOB Putslam_LDB_SOURCES
            "${CMAKE_SOURCE_DIR}/src/LDB/*.cpp"
        )
        file(GLOB Putslam_LDB_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/LDB/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_LDB STATIC ${Putslam_LDB_SOURCES} ${Putslam_LDB_HEADERS})
        else()
                add_library(Putslam_LDB STATIC ${Putslam_LDB_SOURCES} ${Putslam_LDB_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS Putslam_LDB RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_LDB_HEADERS} DESTINATION include/putslam/LDB/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_LDB)

###############################################################################
#
# PUTSLAM RGBD library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_RGBD)
        file(GLOB Putslam_RGBD_SOURCES
            "${CMAKE_SOURCE_DIR}/src/RGBD/*.cpp"
        )
        file(GLOB Putslam_RGBD_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/RGBD/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_RGBD STATIC ${Putslam_RGBD_SOURCES} ${Putslam_RGBD_HEADERS})
        else()
                add_library(Putslam_RGBD STATIC ${Putslam_RGBD_SOURCES} ${Putslam_RGBD_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS Putslam_RGBD RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_RGBD_HEADERS} DESTINATION include/putslam/RGBD/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_RGBD)


###############################################################################
#
# PUTSLAM Matcher library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATCHER)
        file(GLOB Putslam_MATCHER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Matcher/*.cpp"
        )
        file(GLOB Putslam_MATCHER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Matcher/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_Matcher STATIC ${Putslam_MATCHER_SOURCES} ${Putslam_MATCHER_HEADERS})
        else()
                add_library(Putslam_Matcher STATIC ${Putslam_MATCHER_SOURCES} ${Putslam_MATCHER_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_Matcher ${OpenCV_LIBS} Putslam_LDB Putslam_RGBD Putslam_TransformEst)
        INSTALL(TARGETS Putslam_Matcher RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_MATCHER_HEADERS} DESTINATION include/putslam/Matcher/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATCHER)

###############################################################################
#
# PUTSLAM MotionModel library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MOTIONMODEL)
        file(GLOB Putslam_MOTIONMODDEL_SOURCES
            "${CMAKE_SOURCE_DIR}/src/MotionModel/*.cpp"
        )
        file(GLOB Putslam_MOTIONMODDEL_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/MotionModel/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_MotionModel STATIC ${Putslam_MOTIONMODDEL_SOURCES} ${Putslam_MOTIONMODDEL_HEADERS})
        else()
                add_library(Putslam_MotionModel STATIC ${Putslam_MOTIONMODDEL_SOURCES} ${Putslam_MOTIONMODDEL_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS Putslam_MotionModel RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_MOTIONMODDEL_HEADERS} DESTINATION include/putslam/MotionModel/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MOTIONMODEL)

###############################################################################
#
# PUTSLAM Pose Graph (local SLAM) library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)
        file(GLOB Putslam_POSE_GRAPH_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PoseGraph/*.cpp"
        )
        file(GLOB Putslam_POSE_GRAPH_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/PoseGraph/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_PoseGraph STATIC ${Putslam_POSE_GRAPH_SOURCES} ${Putslam_POSE_GRAPH_HEADERS})
        else()
                add_library(Putslam_PoseGraph STATIC ${Putslam_POSE_GRAPH_SOURCES} ${Putslam_POSE_GRAPH_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_PoseGraph csparse g2o_types_slam2d g2o_types_slam3d g2o_csparse_extension g2o_stuff g2o_core g2o_solver_csparse)
        INSTALL(TARGETS Putslam_PoseGraph RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_POSE_GRAPH_HEADERS} DESTINATION include/putslam/PoseGraph/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)


###############################################################################
#
# PUTSLAM visual place recognition
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALPLACERECOGNITION)
        file(GLOB Putslam_VISUALPLACERECOGNITION_SOURCES
            "${CMAKE_SOURCE_DIR}/src/VisualPlaceRecognition/*.cpp"
        )
        file(GLOB Putslam_VISUALPLACERECOGNITION_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/VisualPlaceRecognition/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_VisualPlaceRecognition STATIC ${Putslam_VISUALPLACERECOGNITION_SOURCES} ${Putslam_VISUALPLACERECOGNITION_HEADERS})
        else()
                add_library(Putslam_VisualPlaceRecognition STATIC ${Putslam_VISUALPLACERECOGNITION_SOURCES} ${Putslam_VISUALPLACERECOGNITION_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_VisualPlaceRecognition Putslam_RGBD ${OpenCV_LIBS})
        INSTALL(TARGETS Putslam_VisualPlaceRecognition RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_VISUALPLACERECOGNITION_HEADERS} DESTINATION include/putslam/VisualPlaceRecognition/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALPLACERECOGNITION)



###############################################################################
#
# PUTSLAM LoopClosure library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_LOOPCLOSURE)
        file(GLOB Putslam_LOOPCLOSURE_SOURCES
            "${CMAKE_SOURCE_DIR}/src/LoopClosure/*.cpp"
        )
        file(GLOB Putslam_LOOPCLOSURE_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/LoopClosure/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_LoopClosure STATIC ${Putslam_LOOPCLOSURE_SOURCES} ${Putslam_LOOPCLOSURE_HEADERS})
        else()
                add_library(Putslam_LoopClosure STATIC ${Putslam_LOOPCLOSURE_SOURCES} ${Putslam_LOOPCLOSURE_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_LoopClosure Putslam_Utilities Putslam_Matcher Putslam_RGBD Putslam_VisualPlaceRecognition)
        INSTALL(TARGETS Putslam_LoopClosure RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_LOOPCLOSURE_HEADERS} DESTINATION include/putslam/LoopClosure/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_LOOPCLOSURE)


###############################################################################
#
# PUTSLAM Map library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MAP)
        file(GLOB Putslam_MAP_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Map/*.cpp"
        )
        file(GLOB Putslam_MAP_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Map/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_Map STATIC ${Putslam_MAP_SOURCES} ${Putslam_MAP_HEADERS})
        else()
                add_library(Putslam_Map STATIC ${Putslam_MAP_SOURCES} ${Putslam_MAP_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_Map Putslam_PoseGraph Putslam_Grabber Putslam_Utilities Putslam_Matcher Putslam_TransformEst Putslam_RGBD Putslam_LoopClosure)
        INSTALL(TARGETS Putslam_Map RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_MAP_HEADERS} DESTINATION include/putslam/Map/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MAP)


###############################################################################
#
# PUTSLAM Visualizer library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALIZER)
        file(GLOB Putslam_VISUALIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Visualizer/*.cpp"
        )
        file(GLOB Putslam_VISUALIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Visualizer/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_Visualizer STATIC ${Putslam_VISUALIZER_SOURCES} ${Putslam_VISUALIZER_HEADERS})
        else()
                add_library(Putslam_Visualizer STATIC ${Putslam_VISUALIZER_SOURCES} ${Putslam_VISUALIZER_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(Putslam_Visualizer tinyxml2 Putslam_Map ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY})
        INSTALL(TARGETS Putslam_Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_VISUALIZER_HEADERS} DESTINATION include/putslam/Visualizer/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALIZER)

###############################################################################
#
# PUTSLAM library
#
###############################################################################

if(BUILD_PUTSLAM)
        file(GLOB Putslam_LIBPUTSLAM_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PUTSLAM/*.cpp"
        )
        file(GLOB Putslam_LIBPUTSLAM_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/PUTSLAM/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(Putslam_LIBPUTSLAM STATIC ${Putslam_LIBPUTSLAM_SOURCES} ${Putslam_LIBPUTSLAM_HEADERS})
                TARGET_LINK_LIBRARIES(Putslam_LIBPUTSLAM tinyxml2 Putslam_Map Putslam_Matcher Putslam_MotionModel Putslam_RGBD Putslam_TransformEst Putslam_Utilities ${OpenCV_LIBS} ${catkin_LIBRARIES})
        else()
                add_library(Putslam_LIBPUTSLAM STATIC ${Putslam_LIBPUTSLAM_SOURCES} ${Putslam_LIBPUTSLAM_HEADERS})
                TARGET_LINK_LIBRARIES(Putslam_LIBPUTSLAM tinyxml2 Putslam_Map Putslam_Matcher Putslam_MotionModel Putslam_RGBD Putslam_TransformEst Putslam_Utilities ${OpenCV_LIBS})
        endif(BUILD_ROS)
        INSTALL(TARGETS Putslam_Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${Putslam_LIBPUTSLAM_HEADERS} DESTINATION include/putslam/PUTSLAM/)
endif(BUILD_PUTSLAM)

###############################################################################
#
# PUTSLAM DEMO executableslanguage
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./main.cpp)
        if(BUILD_ROS)
	        rosbuild_add_executable(demo ${DEMO_SOURCES})
        else()
	        add_executable(demo ${DEMO_SOURCES})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(demo tinyxml2 Putslam_Grabber Putslam_Tracker Putslam_TransformEst ${OpenCV_LIBS} Putslam_PoseGraph boost_system)
        INSTALL(TARGETS demo RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)


###############################################################################
#
# PUTSLAM DEMO matching executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./demos/demoMatching.cpp)
        ADD_EXECUTABLE(demoMatching ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoMatching tinyxml2 Putslam_Grabber Putslam_Map Putslam_Matcher Putslam_TransformEst Putslam_LDB ${OpenCV_LIBS} Putslam_PoseGraph Putslam_LIBPUTSLAM boost_system)
        INSTALL(TARGETS demoMatching RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)


###############################################################################
#
# PUTSLAM DEMO g2o graph optimization executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./demos/demoG2o.cpp)
        ADD_EXECUTABLE(demoG2o ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoG2o tinyxml2 Putslam_Grabber Putslam_Tracker Putslam_TransformEst ${OpenCV_LIBS} Putslam_PoseGraph boost_system)
        INSTALL(TARGETS demoG2o RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

###############################################################################
#
# PUTSLAM DEMO g2o graph pruning and optimization executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        ADD_EXECUTABLE(demoPruning ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoPruning tinyxml2 Putslam_Grabber Putslam_Tracker Putslam_TransformEst ${OpenCV_LIBS} Putslam_PoseGraph Putslam_Utilities boost_system)
        INSTALL(TARGETS demoPruning RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

###############################################################################
#
# PUTSLAM DEMO g2o grabber executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_GRABBER)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoGrabber.cpp)
        ADD_EXECUTABLE(demoGrabber ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoGrabber tinyxml2 Putslam_Grabber Putslam_Tracker Putslam_TransformEst ${OpenCV_LIBS} Putslam_PoseGraph boost_system)
        INSTALL(TARGETS demoGrabber RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_GRABBER)

###############################################################################
#
# PUTSLAM DEMO Kabsch
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_KABSCH)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoKabsch.cpp)
        ADD_EXECUTABLE(demoKabsch ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoKabsch tinyxml2 Putslam_Grabber Putslam_Tracker Putslam_TransformEst Putslam_Utilities ${OpenCV_LIBS} Putslam_PoseGraph boost_system)
        INSTALL(TARGETS demoKabsch RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_KABSCH)

###############################################################################
#
# PUTSLAM g2o 2D graph to 3D graph converter
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_GRAPHCONVERTER)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/graph2D2graph3D.cpp)
        ADD_EXECUTABLE(graph2D2graph3D ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(graph2D2graph3D tinyxml2 Putslam_Utilities Putslam_PoseGraph ${OpenCV_LIBS} boost_system)
        INSTALL(TARGETS graph2D2graph3D RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_GRAPHCONVERTER)

###############################################################################
#
# PUTSLAM demo Features map
#
###############################################################################

#if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_MAP)
#        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
#        SET(DEMO_SOURCES ./demos/demoMap.cpp)
#        ADD_EXECUTABLE(demoMap ${DEMO_SOURCES})
#        TARGET_LINK_LIBRARIES(demoMap tinyxml2 Putslam_Utilities Putslam_PoseGraph Putslam_Map Putslam_TransformEst Putslam_LDB ${OpenCV_LIBS} boost_system)
#        INSTALL(TARGETS demoMap RUNTIME DESTINATION bin)
#
#endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_MAP)

###############################################################################
#
# PUTSLAM demo patches
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_PATCH)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoPatches.cpp)
        ADD_EXECUTABLE(demoPatches ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoPatches tinyxml2 Putslam_Utilities Putslam_PoseGraph Putslam_Map Putslam_Matcher Putslam_LDB ${OpenCV_LIBS} boost_system)
        INSTALL(TARGETS demoPatches RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_PATCH)

###############################################################################
#
# PUTSLAM demo Visualizer
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_VISUALIZER)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoVisualizer.cpp)
        ADD_EXECUTABLE(demoVisualizer ${DEMO_SOURCES})
#        add_dependencies(demoVisualizer Putslam_Map)
        TARGET_LINK_LIBRARIES(demoVisualizer Putslam_Visualizer tinyxml2 Putslam_Map Putslam_LIBPUTSLAM ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARY} glut boost_system)
        INSTALL(TARGETS demoVisualizer RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_VISUALIZER)

###############################################################################
#
# PUTSLAM demo Player
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_PLAYER)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoPlayer.cpp)
        ADD_EXECUTABLE(demoPlayer ${DEMO_SOURCES})
#        add_dependencies(demoVisualizer Putslam_Map)
        TARGET_LINK_LIBRARIES(demoPlayer Putslam_Visualizer tinyxml2 Putslam_Map Putslam_LIBPUTSLAM ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARY} glut boost_system)
        INSTALL(TARGETS demoPlayer RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_PLAYER)

###############################################################################
#
# PUTSLAM DEMO ROS executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_ROS AND BUILD_ROS)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./demos/demoROS.cpp)
        rosbuild_add_executable(demoROS ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoROS tinyxml2 Putslam_Grabber Putslam_Map Putslam_Matcher Putslam_TransformEst Putslam_LDB ${OpenCV_LIBS} ${catkin_LIBRARIES} Putslam_PoseGraph LIBPUTSLAM boost_system)
        INSTALL(TARGETS demoROS RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_ROS AND BUILD_ROS)

