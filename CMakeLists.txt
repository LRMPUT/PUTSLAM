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
                rosbuild_add_library(PutslamDEFS STATIC ${DEFS_HEADERS})
        else()
                add_library(PutslamDEFS STATIC ${DEFS_HEADERS})
        endif(BUILD_ROS)
        set_target_properties(PutslamDEFS PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS PUTSLAMDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/putslam/Defs/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_DEFS)

###############################################################################
#
# PUTSLAM Utilities library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_UTILITIES)
        file(GLOB PutslamUTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB PutslamUTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Utilities/*.h"
        )

        if(BUILD_ROS)
                rosbuild_add_library(PutslamUtilities STATIC ${PutslamUTILITIES_SOURCES} ${PutslamUTILITIES_HEADERS})
        else()
                add_library(PutslamUtilities STATIC ${PutslamUTILITIES_SOURCES} ${PutslamTILITIES_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS PutslamUtilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamUtilities_HEADERS} DESTINATION include/putslam/Utilities/)
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
        file(GLOB PutslamGRABBER_SOURCES
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
        file(GLOB PustlamGRABBER_HEADERS
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
                rosbuild_add_library(PutslamGrabber STATIC ${PutslamGRABBER_SOURCES} ${PutslamGRABBER_HEADERS})
                TARGET_LINK_LIBRARIES(PutslamGrabber OpenNI2 ${OpenCV_LIBS} ${catkin_LIBRARIES})
        else()
                add_library(PutslamGrabber STATIC ${PutslamGRABBER_SOURCES} ${PutslamGRABBER_HEADERS})
                TARGET_LINK_LIBRARIES(PutslamGrabber OpenNI2 ${OpenCV_LIBS})
        endif(BUILD_ROS)
        if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            TARGET_LINK_LIBRARIES(PutslamGrabber OpenNI2 ${OpenCV_LIBS} flycapture-c flycapture)
        endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
        file(INSTALL "${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/Lib/" DESTINATION bin)
        INSTALL(TARGETS PutslamGrabber RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamGRABBER_HEADERS}  DESTINATION include/putslam/Grabber/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)

###############################################################################
#
# PUTSLAM Tracker library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRACKER)
        file(GLOB PutslamTRACKER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Tracker/*.cpp"
        )
        file(GLOB PutslamTRACKER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Tracker/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamTracker STATIC ${PutslamTRACKER_SOURCES} ${PutslamTRACKER_HEADERS})
        else()
                add_library(PutslamTracker STATIC ${PutslamTRACKER_SOURCES} ${PutslamTRACKER_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS PutslamTracker RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamTRACKER_HEADERS} DESTINATION include/putslam/Tracker/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRACKER)

###############################################################################
#
# PUTSLAM Transformation Estimation library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)
        file(GLOB PutslamTRANSFORM_ESTIMATOR_SOURCES
            "${CMAKE_SOURCE_DIR}/src/TransformEst/*.cpp"
        )
        file(GLOB PutslamTRANSFORM_ESTIMATOR_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/TransformEst/*.h"
        )

        if(BUILD_ROS)
                rosbuild_add_library(PutslamTransformEst STATIC ${PutslamTRANSFORM_ESTIMATOR_SOURCES} ${PutslamTRANSFORM_ESTIMATOR_HEADERS})
        else()
                add_library(PutslamTransformEst STATIC ${PutslamTRANSFORM_ESTIMATOR_SOURCES} ${PutslamTRANSFORM_ESTIMATOR_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamTransformEst PutslamRGBD)
        INSTALL(TARGETS PutslamTransformEst RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamTRANSFORM_ESTIMATOR_HEADERS} DESTINATION include/putslam/TransformEst/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)

###############################################################################
#
# PUTSLAM USAC library
#
###############################################################################

#if(BUILD_PUTSLAM AND BUILD_PUTSLAM_USAC)
#        file(GLOB PutslamUSAC_SOURCES
#            "${CMAKE_SOURCE_DIR}/src/USAC/*.cpp"
#        )
#        file(GLOB PutslamUSAC_HEADERS
#            "${CMAKE_SOURCE_DIR}/include/putslam/USAC/*.h"
#        )
#
#        ADD_LIBRARY(PutslamUSAC STATIC ${PutslamUSAC_SOURCES} ${PutslamUSAC_HEADERS})
#        TARGET_LINK_LIBRARIES(PutslamUSAC PutslamRGBD)
#        INSTALL(TARGETS PutslamUSAC RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${PutslamUSAC_HEADERS} DESTINATION include/putslam/USAC/)
#endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_USAC)

###############################################################################
#
# PUTSLAM LDB library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_LDB)
        file(GLOB PutslamLDB_SOURCES
            "${CMAKE_SOURCE_DIR}/src/LDB/*.cpp"
        )
        file(GLOB PutslamLDB_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/LDB/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamLDB STATIC ${PutslamLDB_SOURCES} ${PutslamLDB_HEADERS})
        else()
                add_library(PutslamLDB STATIC ${PutslamLDB_SOURCES} ${PutslamLDB_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS PutslamLDB RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamLDB_HEADERS} DESTINATION include/putslam/LDB/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_LDB)

###############################################################################
#
# PUTSLAM RGBD library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_RGBD)
        file(GLOB PutslamRGBD_SOURCES
            "${CMAKE_SOURCE_DIR}/src/RGBD/*.cpp"
        )
        file(GLOB PutslamRGBD_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/RGBD/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamRGBD STATIC ${PutslamRGBD_SOURCES} ${PutslamRGBD_HEADERS})
        else()
                add_library(PutslamRGBD STATIC ${PutslamRGBD_SOURCES} ${PutslamRGBD_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS PutslamRGBD RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamRGBD_HEADERS} DESTINATION include/putslam/RGBD/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_RGBD)


###############################################################################
#
# PUTSLAM Matcher library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATCHER)
        file(GLOB PutslamMATCHER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Matcher/*.cpp"
        )
        file(GLOB PutslamMATCHER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Matcher/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamMatcher STATIC ${PutslamMATCHER_SOURCES} ${PutslamMATCHER_HEADERS})
        else()
                add_library(PutslamMatcher STATIC ${PutslamMATCHER_SOURCES} ${PutslamMATCHER_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamMatcher ${OpenCV_LIBS} PutslamLDB PutslamRGBD PutslamTransformEst)
        INSTALL(TARGETS PutslamMatcher RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamMATCHER_HEADERS} DESTINATION include/putslam/Matcher/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATCHER)

###############################################################################
#
# PUTSLAM MotionModel library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MOTIONMODEL)
        file(GLOB PutslamMOTIONMODDEL_SOURCES
            "${CMAKE_SOURCE_DIR}/src/MotionModel/*.cpp"
        )
        file(GLOB PutslamMOTIONMODDEL_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/MotionModel/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamMotionModel STATIC ${PutslamMOTIONMODDEL_SOURCES} ${PutslamMOTIONMODDEL_HEADERS})
        else()
                add_library(PutslamMotionModel STATIC ${PutslamMOTIONMODDEL_SOURCES} ${PutslamMOTIONMODDEL_HEADERS})
        endif(BUILD_ROS)
        INSTALL(TARGETS PutslamMotionModel RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamMOTIONMODDEL_HEADERS} DESTINATION include/putslam/MotionModel/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MOTIONMODEL)

###############################################################################
#
# PUTSLAM Pose Graph (local SLAM) library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)
        file(GLOB PutslamPOSE_GRAPH_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PoseGraph/*.cpp"
        )
        file(GLOB PutslamPOSE_GRAPH_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/PoseGraph/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamPoseGraph STATIC ${PutslamPOSE_GRAPH_SOURCES} ${PutslamPOSE_GRAPH_HEADERS})
        else()
                add_library(PutslamPoseGraph STATIC ${PutslamPOSE_GRAPH_SOURCES} ${PutslamPOSE_GRAPH_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamPoseGraph csparse g2o_types_slam2d g2o_types_slam3d g2o_csparse_extension g2o_stuff g2o_core g2o_solver_csparse)
        INSTALL(TARGETS PutslamPoseGraph RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamPOSE_GRAPH_HEADERS} DESTINATION include/putslam/PoseGraph/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)


###############################################################################
#
# PUTSLAM visual place recognition
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALPLACERECOGNITION)
        file(GLOB PutslamVISUALPLACERECOGNITION_SOURCES
            "${CMAKE_SOURCE_DIR}/src/VisualPlaceRecognition/*.cpp"
        )
        file(GLOB PutslamVISUALPLACERECOGNITION_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/VisualPlaceRecognition/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamVisualPlaceRecognition STATIC ${PutslamVISUALPLACERECOGNITION_SOURCES} ${PutslamVISUALPLACERECOGNITION_HEADERS})
        else()
                add_library(PutslamVisualPlaceRecognition STATIC ${PutslamVISUALPLACERECOGNITION_SOURCES} ${PutslamVISUALPLACERECOGNITION_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamVisualPlaceRecognition PutslamRGBD ${OpenCV_LIBS})
        INSTALL(TARGETS PutslamVisualPlaceRecognition RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamVISUALPLACERECOGNITION_HEADERS} DESTINATION include/putslam/VisualPlaceRecognition/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALPLACERECOGNITION)



###############################################################################
#
# PUTSLAM LoopClosure library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_LOOPCLOSURE)
        file(GLOB PutslamLOOPCLOSURE_SOURCES
            "${CMAKE_SOURCE_DIR}/src/LoopClosure/*.cpp"
        )
        file(GLOB PutslamLOOPCLOSURE_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/LoopClosure/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamLoopClosure STATIC ${PutslamLOOPCLOSURE_SOURCES} ${PutslamLOOPCLOSURE_HEADERS})
        else()
                add_library(PutslamLoopClosure STATIC ${PutslamLOOPCLOSURE_SOURCES} ${PutslamLOOPCLOSURE_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamLoopClosure PutslamUtilities PutslamMatcher PutslamRGBD PutslamVisualPlaceRecognition)
        INSTALL(TARGETS PutslamLoopClosure RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamLOOPCLOSURE_HEADERS} DESTINATION include/putslam/LoopClosure/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_LOOPCLOSURE)


###############################################################################
#
# PUTSLAM Map library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MAP)
        file(GLOB PutslamMAP_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Map/*.cpp"
        )
        file(GLOB PutslamMAP_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Map/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamMap STATIC ${PutslamMAP_SOURCES} ${PutslamMAP_HEADERS})
        else()
                add_library(PutslamMap STATIC ${PutslamMAP_SOURCES} ${PutslamMAP_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamMap PutslamPoseGraph PutslamGrabber PutslamUtilities PutslamMatcher PutslamTransformEst PutslamRGBD PutslamLoopClosure)
        INSTALL(TARGETS PutslamMap RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamMAP_HEADERS} DESTINATION include/putslam/Map/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MAP)


###############################################################################
#
# PUTSLAM Visualizer library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALIZER)
        file(GLOB PutslamVISUALIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Visualizer/*.cpp"
        )
        file(GLOB PutslamVISUALIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/Visualizer/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamVisualizer STATIC ${PutslamVISUALIZER_SOURCES} ${PutslamVISUALIZER_HEADERS})
        else()
                add_library(PutslamVisualizer STATIC ${PutslamVISUALIZER_SOURCES} ${PutslamVISUALIZER_HEADERS})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(PutslamVisualizer tinyxml2 PutslamMap ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY})
        INSTALL(TARGETS PutslamUtilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamVISUALIZER_HEADERS} DESTINATION include/putslam/Visualizer/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALIZER)

###############################################################################
#
# PUTSLAM library
#
###############################################################################

if(BUILD_PUTSLAM)
        file(GLOB PutslamLIBPUTSLAM_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PUTSLAM/*.cpp"
        )
        file(GLOB PutslamLIBPUTSLAM_HEADERS
            "${CMAKE_SOURCE_DIR}/include/putslam/PUTSLAM/*.h"
        )
        if(BUILD_ROS)
                rosbuild_add_library(PutslamLIBPUTSLAM STATIC ${PutslamLIBPUTSLAM_SOURCES} ${PutslamLIBPUTSLAM_HEADERS})
                TARGET_LINK_LIBRARIES(PutslamLIBPUTSLAM tinyxml2 PutslamMap PutslamMatcher PutslamMotionModel PutslamRGBD PutslamTransformEst PutslamUtilities ${OpenCV_LIBS} ${catkin_LIBRARIES})
        else()
                add_library(PutslamLIBPUTSLAM STATIC ${PutslamLIBPUTSLAM_SOURCES} ${PutslamLIBPUTSLAM_HEADERS})
                TARGET_LINK_LIBRARIES(PutslamLIBPUTSLAM tinyxml2 PutslamMap PutslamMatcher PutslamMotionModel PutslamRGBD PutslamTransformEst PutslamUtilities ${OpenCV_LIBS})
        endif(BUILD_ROS)
        INSTALL(TARGETS PutslamUtilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${PutslamLIBPUTSLAM_HEADERS} DESTINATION include/putslam/PUTSLAM/)
endif(BUILD_PUTSLAM)

###############################################################################
#
# PUTSLAM DEMO executableslanguage
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)

        SET(FILES
                ${DEMO_FILES} /resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./main.cpp)
        if(BUILD_ROS)
	        rosbuild_add_executable(demo ${DEMO_SOURCES})
        else()
	        add_executable(demo ${DEMO_SOURCES})
        endif(BUILD_ROS)
        TARGET_LINK_LIBRARIES(demo tinyxml2 PutslamGrabber PutslamTracker PutslamTransformEst ${OpenCV_LIBS} PutslamPoseGraph boost_system)
        INSTALL(TARGETS demo RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)


###############################################################################
#
# PUTSLAM DEMO matching executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)

        SET(FILES
                ${DEMO_FILES}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./demos/demoMatching.cpp)
        ADD_EXECUTABLE(demoMatching ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoMatching tinyxml2 PutslamGrabber PutslamMap PutslamMatcher PutslamTransformEst PutslamLDB ${OpenCV_LIBS} PutslamPoseGraph PutslamLIBPUTSLAM boost_system)
        INSTALL(TARGETS demoMatching RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)


###############################################################################
#
# PUTSLAM DEMO g2o graph optimization executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

        SET(FILES
                ${DEMO_FILES}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./demos/demoG2o.cpp)
        ADD_EXECUTABLE(demoG2o ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoG2o tinyxml2 PutslamGrabber PutslamTracker PutslamTransformEst ${OpenCV_LIBS} PutslamPoseGraph boost_system)
        INSTALL(TARGETS demoG2o RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

###############################################################################
#
# PUTSLAM DEMO g2o graph pruning and optimization executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

        SET(FILES
                ${DEMO_FILES}/resources/configGlobal.xml
        )

        ADD_EXECUTABLE(demoPruning ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoPruning tinyxml2 PutslamGrabber PutslamTracker PutslamTransformEst ${OpenCV_LIBS} PutslamPoseGraph PutslamUtilities boost_system)
        INSTALL(TARGETS demoPruning RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)

###############################################################################
#
# PUTSLAM DEMO g2o grabber executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_GRABBER)

        SET(FILES
                ${DEMO_FILES}/resources/configGlobal.xml
        )
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoGrabber.cpp)
        ADD_EXECUTABLE(demoGrabber ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoGrabber tinyxml2 PutslamGrabber PutslamTracker PutslamTransformEst ${OpenCV_LIBS} PutslamPoseGraph boost_system)
        INSTALL(TARGETS demoGrabber RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_GRABBER)

###############################################################################
#
# PUTSLAM DEMO Kabsch
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_KABSCH)

        SET(FILES
                ${DEMO_FILES}/resources/configGlobal.xml
        )
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoKabsch.cpp)
        ADD_EXECUTABLE(demoKabsch ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoKabsch tinyxml2 PutslamGrabber PutslamTracker PutslamTransformEst PutslamUtilities ${OpenCV_LIBS} PutslamPoseGraph boost_system)
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
        TARGET_LINK_LIBRARIES(graph2D2graph3D tinyxml2 PutslamUtilities PutslamPoseGraph ${OpenCV_LIBS} boost_system)
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
#        TARGET_LINK_LIBRARIES(demoMap tinyxml2 PutslamUtilities PutslamPoseGraph PutslamMap PutslamTransformEst PutslamLDB ${OpenCV_LIBS} boost_system)
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
        TARGET_LINK_LIBRARIES(demoPatches tinyxml2 PutslamUtilities PutslamPoseGraph PutslamMap PutslamMatcher PutslamLDB ${OpenCV_LIBS} boost_system)
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
#        add_dependencies(demoVisualizer PutslamMap)
        TARGET_LINK_LIBRARIES(demoVisualizer PutslamVisualizer tinyxml2 PutslamMap PutslamLIBPUTSLAM ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARY} glut boost_system)
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
#        add_dependencies(demoVisualizer PutslamMap)
        TARGET_LINK_LIBRARIES(demoPlayer PutslamVisualizer tinyxml2 PutslamMap PutslamLIBPUTSLAM ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARY} glut boost_system)
        INSTALL(TARGETS demoPlayer RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_PLAYER)

###############################################################################
#
# PUTSLAM DEMO ROS executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_ROS AND BUILD_ROS)

        SET(FILES
                ${DEMO_FILES}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./demos/demoROS.cpp)
        rosbuild_add_executable(demoROS ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demoROS tinyxml2 PutslamGrabber PutslamMap PutslamMatcher PutslamTransformEst PutslamLDB ${OpenCV_LIBS} ${catkin_LIBRARIES} PutslamPoseGraph LIBPUTSLAM boost_system)
        INSTALL(TARGETS demoROS RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_ROS AND BUILD_ROS)


