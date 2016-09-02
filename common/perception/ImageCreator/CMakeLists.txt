include_directories(${TEXTILES_INCLUDE_DIRS})

ADD_LIBRARY(ImageCreator HistogramImageCreator.cpp ZBufferDepthImageCreator.cpp RGBImageCreator.cpp)

# Export include path
set(TEXTILES_LIBRARIES ${TEXTILES_LIBRARIES} ImageCreator CACHE INTERNAL "appended libraries")
