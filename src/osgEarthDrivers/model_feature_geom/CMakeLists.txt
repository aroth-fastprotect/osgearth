SET(TARGET_SRC
    FeatureGeomModelSource.cpp )

SET(TARGET_PUBLIC_H 
    FeatureGeomModelOptions )

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/model_feature_geom)
SETUP_PLUGIN(osgearth_model_feature_geom)
