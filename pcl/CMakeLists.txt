cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(FoldingClothes)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(foldingClothes foldingClothes.cpp PointCloudPreprocessor.hpp ZBufferDepthImageCreator.hpp)
target_link_libraries (foldingClothes ${PCL_LIBRARIES})

add_executable(draftCode draftCode.cpp)
target_link_libraries (draftCode ${PCL_LIBRARIES})
