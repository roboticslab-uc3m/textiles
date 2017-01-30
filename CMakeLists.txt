cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Textiles)

find_package(PCL 1.7 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Support for C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Support for OpenMP (parallel processing)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# To get textiles libraries include path
set(TEXTILES_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(TEXTILES_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(TEXTILES_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)


# Subdirectories
add_subdirectory(common)
add_subdirectory(unfolding)
add_subdirectory(unfolding_industrial)
add_subdirectory(ironing)
