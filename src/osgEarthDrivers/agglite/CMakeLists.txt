
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)

SET(TARGET_SRC
	AGGLiteRasterizerTileSource.cpp
	agg.cpp)
	
SET(TARGET_H
	agg.h
)

SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/agglite/AGGLiteOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/agglite)

SETUP_PLUGIN(osgearth_agglite)
