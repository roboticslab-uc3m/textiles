set(TEXTILES_INCLUDE_DIRS ${TEXTILES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/ImageCreator ${CMAKE_CURRENT_SOURCE_DIR}/Preprocessor CACHE INTERNAL "appended header dirs")

# Add libraries
add_subdirectory(Debug)
add_subdirectory(ImageCreator)
add_subdirectory(Preprocessor)

