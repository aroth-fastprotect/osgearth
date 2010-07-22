SET(TARGET_SRC FeatureLabelModelSource.cpp)
SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/model_feature_label/FeatureLabelModelOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/model_feature_label)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_feature_label)


# to install public driver includes:
SET(LIB_NAME model_feature_label)
SET(LIB_PUBLIC_HEADERS FeatureLabelModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

