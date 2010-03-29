
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)

SET(TARGET_SRC
	AGGLiteRasterizerTileSource.cpp
	agg.cpp)
	
SET(TARGET_H
	agg.h
)

SET(TARGET_PUBLIC_H AGGLiteOptions)

SETUP_PLUGIN(osgearth_agglite)
