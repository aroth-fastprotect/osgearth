SET(TARGET_SRC
    FeatureSourceOGR.cpp
    FeatureCursorOGR.cpp
)

SET(TARGET_PUBLIC_H 
	../../../../include/osgEarthDrivers/feature_ogr/OGRFeatureOptions
	../../../../include/osgEarthDrivers/feature_ogr/FeatureCursorOGR
	../../../../include/osgEarthDrivers/feature_ogr/GeometryUtils
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/feature_ogr)

INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY)
SETUP_PLUGIN(osgearth_feature_ogr)

