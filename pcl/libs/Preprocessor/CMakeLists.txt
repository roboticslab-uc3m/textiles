include_directories(${TEXTILES_INCLUDE_DIRS})

ADD_LIBRARY(Preprocessor MeshPreprocessor.cpp PointCloudPreprocessor.cpp)

# Export include path
set(TEXTILES_LIBRARIES ${TEXTILES_LIBRARIES} Preprocessor CACHE INTERNAL "appended libraries")
