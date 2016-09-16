include_directories(${TEXTILES_INCLUDE_DIRS})

ADD_LIBRARY(ImageUtils ImageUtils.cpp)

# Export include path
set(TEXTILES_LIBRARIES ${TEXTILES_LIBRARIES} ImageUtils CACHE INTERNAL "appended libraries")
