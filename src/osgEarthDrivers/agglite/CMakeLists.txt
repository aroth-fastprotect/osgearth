
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)

SET(TARGET_SRC
	AGGLiteRasterizerTileSource.cpp)

SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/agglite/AGGLiteOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/agglite)

SETUP_PLUGIN(osgearth_agglite)
