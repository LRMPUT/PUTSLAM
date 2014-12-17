# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# POSITSOLVER Project configuration
PROJECT(POSITSOLVER)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# core libraries
option(BUILD_POSITSOLVER "Build PUT SLAM libraries" YES)
#option(BUILD_PUTSLAM_DEFS "Build PUTSLAM Defs library" YES)
#mark_as_advanced(BUILD_PUTSLAM_DEFS)
option(BUILD_POSITSOLVER_SOLVER "Build POSITSOLVER Solver library" YES)
mark_as_advanced(BUILD_POSITSOLVER_SOLVER)

# demos
option(BUILD_POSITSOLVER_DEMO "Build POSITSOLVER demonstration programs and libraries" YES)
option(BUILD_POSITSOLVER_DEMO_SOLVER "Build POSITSOLVER solver test" YES)

#additional dependencies
# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
# FIND_PACKAGE(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )
# PCL
# FIND_PACKAGE(PCL REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/g2o/cmake_modules")
# find_package(G2O REQUIRED)

#Eigen
INCLUDE_DIRECTORIES ( "${CMAKE_SOURCE_DIR}/3rdParty" )
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT -DEIGEN_DONT_PARALLELIZE -D_GO_PARALLEL_)

#OpenMP
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()

#Glut
find_package(GLUT)
find_package(OpenGL)

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

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

if (MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

###############################################################################
#
# POSITSOLVER Solver library
#
###############################################################################

if(BUILD_POSITSOLVER AND BUILD_POSITSOLVER_SOLVER)
        file(GLOB SOLVER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Solver/*.cpp"
        )
        file(GLOB SOLVER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Solver/*.h"
        )

        ADD_LIBRARY(Solver STATIC ${SOLVER_SOURCES} ${SOLVER_HEADERS})
        TARGET_LINK_LIBRARIES(Solver ${OPENGL_LIBRARIES} boost_system)
        INSTALL(TARGETS Solver RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${SOLVER_HEADERS} DESTINATION include/PositSolver/Solver/)
endif(BUILD_POSITSOLVER AND BUILD_POSITSOLVER_SOLVER)

###############################################################################
#
# POSITSOLVER demo
#
###############################################################################

if(BUILD_POSITSOLVER_DEMO AND BUILD_POSITSOLVER_DEMO_SOLVER)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./posit_solver_test.cpp)
        ADD_EXECUTABLE(positSolverTest ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(positSolverTest Solver pthread)
        INSTALL(TARGETS positSolverTest RUNTIME DESTINATION bin)

endif(BUILD_POSITSOLVER_DEMO AND BUILD_POSITSOLVER_DEMO_SOLVER)
