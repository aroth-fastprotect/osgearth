SET(TARGET_SRC
    FeatureSourceOGR.cpp
    FeatureCursorOGR.cpp
)

SET(TARGET_PUBLIC_H 
	FeatureCursorOGR
    OGRFeatureOptions
)

INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/feature_ogr)
SETUP_PLUGIN(osgearth_feature_ogr)

