# Add programs
include_directories(${TEXTILES_INCLUDE_DIRS})

add_executable(ironingClothes ironingClothes.cpp)
target_link_libraries (ironingClothes ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

