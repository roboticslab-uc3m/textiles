cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(FoldingClothes)

find_package(PCL 1.7 REQUIRED)

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

# To get textiles libraries include path
set(TEXTILES_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(TEXTILES_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(TEXTILES_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)

set(TEXTILES_INCLUDE_DIRS ${TEXTILES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE INTERNAL "appended header dirs")
add_subdirectory(libs)

# Add programs
include_directories(${TEXTILES_INCLUDE_DIRS})

add_executable(foldingClothes foldingClothes.cpp)
target_link_libraries (foldingClothes ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(foldingClothesMesh foldingClothesMesh.cpp)
target_link_libraries (foldingClothesMesh ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(draftCode draftCode.cpp)
target_link_libraries (draftCode ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(create_test_cube testcube.cpp)
target_link_libraries (create_test_cube ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(clusteringCleanup clusteringCleanup.cpp)
target_link_libraries (clusteringCleanup ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(render2png render2png.cpp)
target_link_libraries (render2png ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

