# Add programs
include_directories(${TEXTILES_INCLUDE_DIRS})

add_executable(ironingClothes ironingClothes.cpp)
target_link_libraries (ironingClothes ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(wrinkleDetection wrinkleDetection.cpp)
target_link_libraries (wrinkleDetection ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(testRegionGrowing testRegionGrowing.cpp)
target_link_libraries (testRegionGrowing ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

