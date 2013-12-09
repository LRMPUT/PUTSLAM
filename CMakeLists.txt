# cmake requirements
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Estimator of Hand Configuration
PROJECT(PUTSLAM)

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# core libraries
option(BUILD_PUTSLAM "Build PUT SLAM libraries" YES)
option(BUILD_PUTSLAM_DEFS "Build PUTSLAM Defs library" YES)
mark_as_advanced(BUILD_PUTSLAM_DEFS)
option(BUILD_PUTSLAM_MATH "Build HandEst Math library" YES)
mark_as_advanced(BUILD_PUTSLAM_MATH)
option(BUILD_PUTSLAM_GRABBER "Build HandEst Grabber library" YES)
mark_as_advanced(BUILD_PUTSLAM_GRABBER)

# demos
option(BUILD_PUTSLAM_DEMO "Build PUT SLAM demonstration programs and libraries" YES)
option(BUILD_PUTSLAM_DEMO_BASIC "Build basic PUT SLAM test" YES)
mark_as_advanced(BUILD_PUTSLAM_DEMO_BASIC)

#additional dependencies
# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# PCL
#FIND_PACKAGE(PCL REQUIRED)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_PARENT}/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PARENT}/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_PARENT}/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
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

###############################################################################
#
# PUTSLAM defs library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_DEFS)
	file(GLOB DEFS_HEADERS
	    "${CMAKE_SOURCE_DIR}/include/Defs/putslam_defs.h"
	)
        ADD_LIBRARY(PUTSLAM_DEFS STATIC ${DEFS_HEADERS})
        set_target_properties(PUTSLAM_DEFS PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS PUTSLAMDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Defs/)

endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_DEFS)

###############################################################################
#
# PUTSLAM Math library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATH)
	file(GLOB MATH_SOURCES
	    "${CMAKE_SOURCE_DIR}/src/Core/Math/*.cpp"
	)
	file(GLOB MATH_HEADERS
	    "${CMAKE_SOURCE_DIR}/include/Core/Math/*.h"
	)

	ADD_LIBRARY(Math STATIC ${MATH_SOURCES} ${MATH_HEADERS})
	#TARGET_LINK_LIBRARIES(Math)
	INSTALL(TARGETS Math RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${MATH_HEADERS} DESTINATION include/handest/Core/Math/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATH)

###############################################################################
#
# PUTSLAM Grabber library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)
	SET(GRABBER_HEADERS ./include/Grabber/grabber.h)

	# Kinect Grabber
	SET(GRABBERKINECT_SOURCES ./src/Grabber/kinect_grabber.cpp)
	SET(GRABBERKINECT_HEADERS ./include/Grabber/kinect_grabber.h)
	ADD_LIBRARY(GrabberKinect STATIC ${GRABBERKINECT_SOURCES} ${GRABBER_HEADERS} ${GRABBERKINECT_HEADERS})
	#TARGET_LINK_LIBRARIES(GrabberKinect)
	INSTALL(TARGETS GrabberKinect RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${GRABBER_HEADERS} DESTINATION include/handest/Grabber/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)


###############################################################################
#
# PUTSLAM DEMO executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)
	SET(DEMO_SOURCES ./main.cpp)
	ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
	TARGET_LINK_LIBRARIES(Demo Math GrabberKinect ${OpenCV_LIBS})
	INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)
