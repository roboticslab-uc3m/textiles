set(TEXTILES_INCLUDE_DIRS ${TEXTILES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")

include_directories(${TEXTILES_INCLUDE_DIRS})

ADD_LIBRARY(Debug Debug.cpp)

# Export include path
set(TEXTILES_LIBRARIES ${TEXTILES_LIBRARIES} Debug CACHE INTERNAL "appended libraries")

# Tests:
add_executable(test_Debug test_Debug.cpp)
target_link_libraries (test_Debug ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})
