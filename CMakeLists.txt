# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# PUTSLAM Project configuration
PROJECT(PUTSLAM)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

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
option(BUILD_PUTSLAM_POSE_GRAPH "Build PUTSLAM Pose Graph (local and global graph) library" ON)
mark_as_advanced(BUILD_PUTSLAM_POSE_GRAPH)
option(BUILD_PUTSLAM_MAP "Build PUTSLAM Map library" ON)
mark_as_advanced(BUILD_PUTSLAM_MAP)
option(BUILD_PUTSLAM_PATCH "Build PUTSLAM Patches" ON)
mark_as_advanced(BUILD_PUTSLAM_PATCH)
option(BUILD_PUTSLAM_VISUALIZER "Build PUTSLAM Visualizer" ON)
mark_as_advanced(BUILD_PUTSLAM_VISUALIZER)

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

#additional dependencies

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

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

#g2o library
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
link_directories(${G2O_LIBRARY_DIRS})

#CSparse library
FIND_PACKAGE(CSparse)
include_directories(${LIBCSPARSE_INCLUDE_DIR})
link_directories(${LIBCSPARSE_LIBRARY})

#Eigen
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#QGLViewer library
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
# PUTSLAM Utilities library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_UTILITIES)
        file(GLOB UTILITIES_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Utilities/*.cpp"
        )
        file(GLOB UTILITIES_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Utilities/*.h"
        )

        ADD_LIBRARY(Utilities STATIC ${UTILITIES_SOURCES} ${UTILITIES_HEADERS})
        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
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
        file(GLOB GRABBER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Grabber/calibration.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/depthSensorModel.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/kinectGrabber.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/fileGrabber.cpp"
            #if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
                "${CMAKE_SOURCE_DIR}/src/Grabber/ptgreyGrabber.cpp"
            #endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            "${CMAKE_SOURCE_DIR}/src/Grabber/templateGrabber.cpp"
            "${CMAKE_SOURCE_DIR}/src/Grabber/xtionGrabber.cpp"
        )
        file(GLOB GRABBER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Grabber/grabber.h"
            "${CMAKE_SOURCE_DIR}/include/Grabber/calibration.h"
            "${CMAKE_SOURCE_DIR}/include/Grabber/depthSensorModel.h"
            "${CMAKE_SOURCE_DIR}/include/Grabber/kinectGrabber.h"
            "${CMAKE_SOURCE_DIR}/include/Grabber/fileGrabber.h"
            #if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
                "${CMAKE_SOURCE_DIR}/include/Grabber/ptgreyGrabber.h"
            #endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            "${CMAKE_SOURCE_DIR}/include/Grabber/templateGrabber.h"
            "${CMAKE_SOURCE_DIR}/include/Grabber/xtionGrabber.h"
        )

        ADD_LIBRARY(Grabber STATIC ${GRABBER_SOURCES} ${GRABBER_HEADERS})
        TARGET_LINK_LIBRARIES(Grabber OpenNI2 ${OpenCV_LIBS})
        if(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
            TARGET_LINK_LIBRARIES(Grabber OpenNI2 ${OpenCV_LIBS} flycapture-c flycapture)
        endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_GRABBER AND BUILD_PUTSLAM_GRABBER_PTGREY)
        file(INSTALL "${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/Lib/" DESTINATION bin)
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
        TARGET_LINK_LIBRARIES(TransformEst RGBD)
        INSTALL(TARGETS TransformEst RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${TRANSFORM_ESTIMATOR_HEADERS} DESTINATION include/putslam/TransformEst/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_TRANSFORM_ESTIMATOR)

###############################################################################
#
# PUTSLAM LDB library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_LDB)
        file(GLOB LDB_SOURCES
            "${CMAKE_SOURCE_DIR}/src/LDB/*.cpp"
        )
        file(GLOB LDB_HEADERS
            "${CMAKE_SOURCE_DIR}/include/LDB/*.h"
        )

        ADD_LIBRARY(LDB STATIC ${LDB_SOURCES} ${LDB_HEADERS})
        INSTALL(TARGETS LDB RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${LDB_HEADERS} DESTINATION include/putslam/LDB/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_LDB)

###############################################################################
#
# PUTSLAM RGBD library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_RGBD)
        file(GLOB RGBD_SOURCES
            "${CMAKE_SOURCE_DIR}/src/RGBD/*.cpp"
        )
        file(GLOB MATCHER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/RGBD/*.h"
        )

        ADD_LIBRARY(RGBD STATIC ${RGBD_SOURCES} ${RGBD_HEADERS})
        INSTALL(TARGETS RGBD RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${RGBD_HEADERS} DESTINATION include/putslam/RGBD/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_RGBD)


###############################################################################
#
# PUTSLAM Matcher library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATCHER)
        file(GLOB MATCHER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Matcher/*.cpp"
        )
        file(GLOB MATCHER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Matcher/*.h"
        )

        ADD_LIBRARY(Matcher STATIC ${MATCHER_SOURCES} ${MATCHER_HEADERS})
        TARGET_LINK_LIBRARIES(TransformEst LDB RGBD)
        INSTALL(TARGETS Matcher RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${MATCHER_HEADERS} DESTINATION include/putslam/Matcher/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MATCHER)

###############################################################################
#
# PUTSLAM MotionModel library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MOTIONMODEL)
        file(GLOB MOTIONMODDEL_SOURCES
            "${CMAKE_SOURCE_DIR}/src/MotionModel/*.cpp"
        )
        file(GLOB MOTIONMODDEL_HEADERS
            "${CMAKE_SOURCE_DIR}/include/MotionModel/*.h"
        )

        ADD_LIBRARY(MotionModel STATIC ${MOTIONMODDEL_SOURCES} ${MOTIONMODDEL_HEADERS})
      	INSTALL(TARGETS MotionModel RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${MOTIONMODDEL_HEADERS} DESTINATION include/putslam/MotionModel/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MOTIONMODEL)

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
        TARGET_LINK_LIBRARIES(PoseGraph csparse g2o_types_slam2d g2o_types_slam3d g2o_csparse_extension g2o_stuff g2o_core g2o_solver_csparse)
        INSTALL(TARGETS PoseGraph RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${POSE_GRAPH_HEADERS} DESTINATION include/putslam/PoseGraph/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_POSE_GRAPH)

###############################################################################
#
# PUTSLAM Map library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_MAP)
        file(GLOB MAP_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Map/*.cpp"
        )
        file(GLOB MAP_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Map/*.h"
        )

        ADD_LIBRARY(Map STATIC ${MAP_SOURCES} ${MAP_HEADERS})
        TARGET_LINK_LIBRARIES(Map PoseGraph Grabber)
        INSTALL(TARGETS Map RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${MAP_HEADERS} DESTINATION include/putslam/Map/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_MAP)

###############################################################################
#
# PUTSLAM Visualizer library
#
###############################################################################

if(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALIZER)
        file(GLOB VISUALIZER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Visualizer/*.cpp"
        )
        file(GLOB VISUALIZER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Visualizer/*.h"
        )
        ADD_LIBRARY(Visualizer STATIC ${VISUALIZER_SOURCES} ${VISUALIZER_HEADERS})
        TARGET_LINK_LIBRARIES(Visualizer tinyxml2 Map ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY})
        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/putslam/Visualizer/)
endif(BUILD_PUTSLAM AND BUILD_PUTSLAM_VISUALIZER)

###############################################################################
#
# PUTSLAM library
#
###############################################################################

if(BUILD_PUTSLAM)
        file(GLOB LIBPUTSLAM_SOURCES
            "${CMAKE_SOURCE_DIR}/src/PUTSLAM/*.cpp"
        )
        file(GLOB LIBPUTSLAM_HEADERS
            "${CMAKE_SOURCE_DIR}/include/PUTSLAM/*.h"
        )
        ADD_LIBRARY(LIBPUTSLAM STATIC ${LIBPUTSLAM_SOURCES} ${LIBPUTSLAM_HEADERS})
        TARGET_LINK_LIBRARIES(LIBPUTSLAM tinyxml2 Map Matcher MotionModel RGBD TransformEst Utilities ${OpenCV_LIBS})
        INSTALL(TARGETS Utilities RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${LIBPUTSLAM_HEADERS} DESTINATION include/putslam/PUTSLAM/)
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
        ADD_EXECUTABLE(demo ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demo tinyxml2 Grabber Tracker TransformEst ${OpenCV_LIBS} PoseGraph boost_system)
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
        TARGET_LINK_LIBRARIES(demoMatching tinyxml2 Grabber Map Matcher TransformEst LDB ${OpenCV_LIBS} PoseGraph LIBPUTSLAM boost_system)
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
        add_dependencies(demoG2o g2o)
        TARGET_LINK_LIBRARIES(demoG2o tinyxml2 Grabber Tracker TransformEst ${OpenCV_LIBS} PoseGraph boost_system)
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

        SET(DEMO_SOURCES ./demos/demoPruning.cpp)
        ADD_EXECUTABLE(demoPruning ${DEMO_SOURCES})
        add_dependencies(demoPruning g2o)
        TARGET_LINK_LIBRARIES(demoPruning tinyxml2 Grabber Tracker TransformEst ${OpenCV_LIBS} PoseGraph Utilities boost_system)
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
        add_dependencies(demoGrabber g2o)
        TARGET_LINK_LIBRARIES(demoGrabber tinyxml2 Grabber Tracker TransformEst ${OpenCV_LIBS} PoseGraph boost_system)
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
        add_dependencies(demoKabsch g2o)
        TARGET_LINK_LIBRARIES(demoKabsch tinyxml2 Grabber Tracker TransformEst Utilities ${OpenCV_LIBS} PoseGraph boost_system)
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
        add_dependencies(graph2D2graph3D g2o)
        TARGET_LINK_LIBRARIES(graph2D2graph3D tinyxml2 Utilities PoseGraph ${OpenCV_LIBS} boost_system)
        INSTALL(TARGETS graph2D2graph3D RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_GRAPHCONVERTER)

###############################################################################
#
# PUTSLAM demo Features map
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_MAP)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoMap.cpp)
        ADD_EXECUTABLE(demoMap ${DEMO_SOURCES})
        add_dependencies(demoMap g2o)
        TARGET_LINK_LIBRARIES(demoMap tinyxml2 Utilities PoseGraph Map LDB ${OpenCV_LIBS} boost_system)
        INSTALL(TARGETS demoMap RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_MAP)

###############################################################################
#
# PUTSLAM demo patches
#
###############################################################################

if(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_PATCH)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./demos/demoPatches.cpp)
        ADD_EXECUTABLE(demoPatches ${DEMO_SOURCES})
        add_dependencies(demoPatches g2o)
        TARGET_LINK_LIBRARIES(demoPatches tinyxml2 Utilities PoseGraph Map Matcher LDB ${OpenCV_LIBS} boost_system)
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
#        add_dependencies(demoVisualizer Map)
        TARGET_LINK_LIBRARIES(demoVisualizer Visualizer tinyxml2 Map ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARY} boost_system)
        INSTALL(TARGETS demoVisualizer RUNTIME DESTINATION bin)

endif(BUILD_PUTSLAM_DEMO AND BUILD_PUTSLAM_DEMO_VISUALIZER)
