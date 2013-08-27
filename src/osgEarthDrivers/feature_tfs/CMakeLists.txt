SET(TARGET_SRC
    FeatureSourceTFS.cpp    
)

SET(TARGET_PUBLIC_H
    TFSFeatureOptions
)

INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)
SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/feature_tfs)
SETUP_PLUGIN(osgearth_feature_tfs)
