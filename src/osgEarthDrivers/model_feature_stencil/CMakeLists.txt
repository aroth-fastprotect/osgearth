SET(TARGET_SRC
    FeatureStencilModelSource.cpp
)

SET(TARGET_PUBLIC_H
    ../../../../include/osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/model_feature_stencil)

SET(TARGET_COMMON_LIBRARIES
    ${TARGET_COMMON_LIBRARIES}
    osgEarthFeatures
    osgEarthSymbology
)

SETUP_PLUGIN(osgearth_model_feature_stencil)
