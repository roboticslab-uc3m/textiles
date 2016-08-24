# Add programs
include_directories(${TEXTILES_INCLUDE_DIRS})

add_executable(ironingClothes ironingClothes.cpp)
target_link_libraries (ironingClothes ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(testRegionGrowing testRegionGrowing.cpp)
target_link_libraries (testRegionGrowing ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
add_executable(wrinkleDetection wrinkleDetection.cpp)
target_link_libraries (wrinkleDetection ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})
