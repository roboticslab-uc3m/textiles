include_directories(${TEXTILES_INCLUDE_DIRS})

ADD_LIBRARY(ImageCreator ImageCreator.cpp HistogramImageCreator.cpp ZBufferDepthImageCreator.cpp RGBDImageCreator.cpp MaskImageCreator.cpp DepthImageCreator.cpp)

# Export include path
set(TEXTILES_LIBRARIES ${TEXTILES_LIBRARIES} ImageCreator CACHE INTERNAL "appended libraries")
