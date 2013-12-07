# cmake requirements
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# Estimator of Hand Configuration
PROJECT(HandEstimator)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/dependencies")

# Math
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

# Grabber interface
SET(GRABBER_HEADERS ./include/Grabber/grabber.h)

# Kinect Grabber
SET(GRABBERKINECT_SOURCES ./src/Grabber/kinect_grabber.cpp)
SET(GRABBERKINECT_HEADERS ./include/Grabber/kinect_grabber.h)
ADD_LIBRARY(GrabberKinect STATIC ${GRABBERKINECT_SOURCES} ${GRABBER_HEADERS} ${GRABBERKINECT_HEADERS})
#TARGET_LINK_LIBRARIES(GrabberKinect)
INSTALL(TARGETS GrabberKinect RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${GRABBER_HEADERS} DESTINATION include/handest/Grabber/)

# Filter interface
SET(FILTER_HEADERS ./include/Filter/filter.h)

# Filter PCL
SET(FILTERPCL_SOURCES ./src/Filter/filterPCL.cpp)
SET(FILTERPCL_HEADERS ./include/Filter/filterPCL.h)
ADD_LIBRARY(Filter STATIC ${FILTERPCL_SOURCES} ${FILTER_HEADERS} ${FILTERPCL_HEADERS})
#TARGET_LINK_LIBRARIES(FilterPCL)
INSTALL(TARGETS Filter RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${FILTER_HEADERS} DESTINATION include/handest/Filter/)

# OptimizationFunction interface 
SET(OPTIMIZATIONFUNCTION_HEADERS ./include/OptimizationFunction/optimizationFunction.h)

# OptimizationFunctionGauss
SET(OPTIMIZATIONFUNCTIONGAUSS_SOURCES ./src/OptimizationFunction/optimizationFunctionGauss.cpp)
SET(OPTIMIZATIONFUNCTIONGAUSS_HEADERS ./include/OptimizationFunction/optimizationFunctionGauss.h)
SET(OPTIMIZATIONFUNCTIONPF_SOURCES ./src/OptimizationFunction/optimizationFunctionPF.cpp)
SET(OPTIMIZATIONFUNCTIONPF_HEADERS ./include/OptimizationFunction/optimizationFunctionPF.h)
ADD_LIBRARY(OptimizationFunction STATIC ${OPTIMIZATIONFUNCTIONGAUSS_SOURCES} ${OPTIMIZATIONFUNCTION_HEADERS} ${OPTIMIZATIONFUNCTIONGAUSS_HEADERS} ${OPTIMIZATIONFUNCTIONPF_HEADERS} ${OPTIMIZATIONFUNCTIONPF_SOURCES})
#TARGET_LINK_LIBRARIES(OptimizationFunction)
INSTALL(TARGETS OptimizationFunction RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)

# Visualizer interface
SET(VISUALIZER_HEADERS ./include/Visualizer/visualizer.h)

# OpenGL Visualizer
SET(VISUALIZERGL_SOURCES ./src/Visualizer/visualizerGL.cpp)
SET(VISUALIZERGL_HEADERS ./include/Visualizer/visualizerGL.h)
ADD_LIBRARY(VisualizerGL STATIC ${VISUALIZERGL_SOURCES} ${VISUALIZER_HEADERS} ${VISUALIZERGL_HEADERS})
#TARGET_LINK_LIBRARIES(VisualizerGL)
INSTALL(TARGETS VisualizerGL RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/handest/Visualizer/)

# Kinematic interface
SET(KINEMATIC_HEADERS ./include/Kinematic/forward_kinematics.h)

# Forward Kinematics
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)
SET(FK_SOURCES ./src/Kinematic/kinematic_liego.cpp)
SET(FK_HEADERS ./include/Kinematic/kinematic_liego.h )
ADD_LIBRARY(ForwardKinematics STATIC ${KINEMATIC_HEADERS} ${FK_SOURCES} ${FK_HEADERS})
INSTALL(TARGETS ForwardKinematics RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)

#Optimization Interface
SET(OPTIMIZATION_HEADERS ./include/Optimization/optimization.h)

#Optimization 
SET(OPTIMIZATION_SOURCES ./src/Optimization/optimizationPSO.cpp)
SET(OPTIMIZATIONPSO_HEADERS ./include/Optimization/optimizationPSO.h)
ADD_LIBRARY(Optimization STATIC ${OPTIMIZATION_SOURCES} ${OPTIMIZATION_HEADERS} ${OPTIMIZATIONPSO_HEADERS})
INSTALL(TARGETS Optimization RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)


# Demo
SET(DEMO_SOURCES ./main.cpp)
ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
TARGET_LINK_LIBRARIES(Demo Math GrabberKinect ForwardKinematics Optimization OptimizationFunction Filter VisualizerGL)
INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
