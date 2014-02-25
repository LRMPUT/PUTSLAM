# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# PUTSLAM Project configuration
PROJECT(PUTSLAM)
SET(CMAKE_CXX_FLAGS "-std=c++0x -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# core libraries
option(BUILD_PUTSLAM "Build PUT SLAM libraries" YES)
option(BUILD_PUTSLAM_G2O "Build g2o library" YES)
mark_as_advanced(BUILD_PUTSLAM_G2O)
option(BUILD_PUTSLAM_DEFS "Build PUTSLAM Defs library" YES)
mark_as_advanced(BUILD_PUTSLAM_DEFS)
option(BUILD_PUTSLAM_GRABBER "Build PUTSLAM Grabber library" YES)
mark_as_advanced(BUILD_PUTSLAM_GRABBER)
option(BUILD_PUTSLAM_TRACKER "Build PUTSLAM Tracker library" YES)
mark_as_advanced(BUILD_PUTSLAM_TRACKER)
option(BUILD_PUTSLAM_TRANSFORM_ESTIMATOR "Build PUTSLAM Transformation Estimation library" YES)
mark_as_advanced(BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)
option(BUILD_PUTSLAM_POSE_GRAPH "Build PUTSLAM Pose Graph (local and global graph) library" YES)
mark_as_advanced(BUILD_PUTSLAM_POSE_GRAPH)
option(BUILD_PUTSLAM_GLOBAL_MAP "Build PUTSLAM Global Map library" YES)
mark_as_advanced(BUILD_PUTSLAM_GLOBAL_MAP)

# demos
option(BUILD_PUTSLAM_DEMO "Build PUT SLAM demonstration programs and libraries" YES)
option(BUILD_PUTSLAM_DEMO_G2O "Build g2o PUT SLAM test" YES)
mark_as_advanced(BUILD_PUTSLAM_DEMO_G2O)
option(BUILD_PUTSLAM_DEMO_BASIC "Build basic PUT SLAM test" YES)
mark_as_advanced(BUILD_PUTSLAM_DEMO_BASIC)

#additional dependencies
# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# PCL
FIND_PACKAGE(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/g2o/cmake_modules")
find_package(G2O REQUIRED)

#Eigen
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#g2o
link_directories("${CMAKE_SOURCE_DIR}/build/lib")
link_directories("${CMAKE_SOURCE_DIR}/build/bin")

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

###############################################################################
#
# PUTSLAM g2o library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_G2O)
        ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/g2o)
        INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/g2o")
        INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/g2o/EXTERNAL/ceres")
        INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/g2o/EXTERNAL/csparse")
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_G2O)

###############################################################################
#
# PUTSLAM tinyXML2 library
#
###############################################################################

add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

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
# PUTSLAM Grabber library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)
        file(GLOB GRABBER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Grabber/*.cpp"
        )
        file(GLOB GRABBER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Grabber/*.h"
        )

        ADD_LIBRARY(Grabber STATIC ${GRABBER_SOURCES} ${GRABBER_HEADERS})
        INSTALL(TARGETS Grabber RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${GRABBER_HEADERS} GRABBERKINECT_HEADERS DESTINATION include/putslam/Grabber/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER)

###############################################################################
#
# PUTSLAM Tracker library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRACKER)
        file(GLOB TRACKER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Tracker/*.cpp"
        )
        file(GLOB TRACKER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Tracker/*.h"
        )

        ADD_LIBRARY(Tracker STATIC ${TRACKER_SOURCES} ${TRACKER_HEADERS})
        INSTALL(TARGETS Tracker RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${TRACKER_HEADERS} DESTINATION include/putslam/Tracker/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRACKER)

###############################################################################
#
# PUTSLAM Transformation Estimation library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)
        file(GLOB TRANSFORM_ESTIMATOR_SOURCES
            "${CMAKE_SOURCE_DIR}/src/TransformEst/*.cpp"
        )
        file(GLOB TRANSFORM_ESTIMATOR_HEADERS
            "${CMAKE_SOURCE_DIR}/include/TransformEst/*.h"
        )

        ADD_LIBRARY(TransformEst STATIC ${TRANSFORM_ESTIMATOR_SOURCES} ${TRANSFORM_ESTIMATOR_HEADERS})
        INSTALL(TARGETS TransformEst RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${TRANSFORM_ESTIMATOR_HEADERS} DESTINATION include/putslam/TransformEst/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)

###############################################################################
#
# PUTSLAM Pose Graph (local SLAM) library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)
        file(GLOB POSE_GRAPH_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PoseGraph/*.cpp"
        )
        file(GLOB POSE_GRAPH_HEADERS
            "${CMAKE_SOURCE_DIR}/include/PoseGraph/*.h"
        )

        ADD_LIBRARY(PoseGraph STATIC ${POSE_GRAPH_SOURCES} ${POSE_GRAPH_HEADERS})
        TARGET_LINK_LIBRARIES(PoseGraph core csparse csparse_extension solver_csparse types_slam2d types_slam3d g2o_types_slam2d g2o_types_slam3d g2o_csparse_extension g2o_ext_csparse g2o_stuff g2o_core g2o_solver_csparse)
        INSTALL(TARGETS PoseGraph RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${POSE_GRAPH_HEADERS} DESTINATION include/putslam/PoseGraph/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)

###############################################################################
#
# PUTSLAM Global Map library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GLOBAL_MAP)
        file(GLOB GLOBAL_MAP_SOURCES
            "${CMAKE_SOURCE_DIR}/src/GlobalMap/*.cpp"
        )
        file(GLOB GLOBAL_MAP_HEADERS
            "${CMAKE_SOURCE_DIR}/include/GlobalMap/*.h"
        )

        ADD_LIBRARY(GlobalMap STATIC ${GLOBAL_MAP_SOURCES} ${GLOBAL_MAP_HEADERS})
        INSTALL(TARGETS GlobalMap RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${GLOBAL_MAP_HEADERS} DESTINATION include/putslam/GlobalMap/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GLOBAL_MAP)

###############################################################################
#
# PUTSLAM DEMO executables
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_BASIC)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(Demo tinyxml2 Grabber Tracker TransformEst ${OpenCV_LIBS} PoseGraph GlobalMap boost_system)
        if(BUILD_PUTSLAM AND BUILD_PUTSLAM_G2O)
            ADD_CUSTOM_COMMAND( TARGET Demo POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/3rdParty/g2o/lib/" "${PROJECT_SOURCE_DIR}/build/lib"
                    COMMENT "copying g2o libs ..."
            )
        endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_G2O)
        INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
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

        SET(DEMO_SOURCES ./demo_g2o.cpp)
        ADD_EXECUTABLE(Demo_g2o ${DEMO_SOURCES})
        add_dependencies(Demo_g2o g2o)
        TARGET_LINK_LIBRARIES(Demo_g2o tinyxml2 Grabber Tracker TransformEst ${OpenCV_LIBS} PoseGraph GlobalMap boost_system)
        if(BUILD_PUTSLAM AND BUILD_PUTSLAM_G2O)
            ADD_CUSTOM_COMMAND( TARGET Demo_g2o POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/3rdParty/g2o/lib/" "${PROJECT_SOURCE_DIR}/build/lib"
                    COMMENT "copying g2o libs ..."
            )
        endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_G2O)
        INSTALL(TARGETS Demo_g2o RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_G2O)
