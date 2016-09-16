set(TEXTILES_INCLUDE_DIRS ${TEXTILES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/ImageCreator ${CMAKE_CURRENT_SOURCE_DIR}/ImageUtils  CACHE INTERNAL "appended header dirs")

add_subdirectory(ImageCreator)
add_subdirectory(ImageUtils)
