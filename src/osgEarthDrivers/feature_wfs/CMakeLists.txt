SET(TARGET_SRC
    FeatureSourceWFS.cpp    
)

SET(TARGET_H   
    WFSFeatureOptions
)

INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)
SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY)
SETUP_PLUGIN(osgearth_feature_wfs)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
