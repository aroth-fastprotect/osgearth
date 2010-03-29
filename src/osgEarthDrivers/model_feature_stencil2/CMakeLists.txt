SET(TARGET_SRC
    FeatureStencilModelSource.cpp
)

SET(TARGET_PUBLIC_H
    FeatureStencilModelOptions
)
    
SET(TARGET_COMMON_LIBRARIES
    ${TARGET_COMMON_LIBRARIES}
    osgEarthFeatures2
    osgEarthSymbology
)

SETUP_PLUGIN(osgearth_model_feature_stencil2)
