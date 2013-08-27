
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)

SET(TARGET_SRC
	AGGLiteRasterizerTileSource.cpp)

SET(TARGET_PUBLIC_H AGGLiteOptions)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/agglite)

SETUP_PLUGIN(osgearth_agglite)
