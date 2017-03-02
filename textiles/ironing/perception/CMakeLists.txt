# Add programs
find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS} ${TEXTILES_INCLUDE_DIRS})

add_executable(garmentSegmentation garmentSegmentation.cpp)
target_link_libraries (garmentSegmentation ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(garmentCleanup garmentCleanup.cpp)
target_link_libraries (garmentCleanup ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(testRegionGrowing testRegionGrowing.cpp)
target_link_libraries (testRegionGrowing ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(wrinkleDetection wrinkleDetection.cpp)
target_link_libraries (wrinkleDetection ${PCL_LIBRARIES} ${YARP_LIBRARIES} ${TEXTILES_LIBRARIES})
