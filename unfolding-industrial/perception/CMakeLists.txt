# Add programs
include_directories(${TEXTILES_INCLUDE_DIRS})

add_executable(kinfuUnfolding kinfuUnfolding.cpp)
target_link_libraries (kinfuUnfolding ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})
