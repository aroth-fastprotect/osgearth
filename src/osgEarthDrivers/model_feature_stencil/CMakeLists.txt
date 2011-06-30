SET(TARGET_SRC
    FeatureStencilModelSource.cpp
)

SET(TARGET_PUBLIC_H
    FeatureStencilModelOptions
)

SET(TARGET_COMMON_LIBRARIES
    ${TARGET_COMMON_LIBRARIES}
    osgEarthFeatures
    osgEarthSymbology
)

FILE(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/\$(TargetFileName).manifest" _manifestfile)
SET(TARGET_LDFLAGS "/MANIFEST /MANIFESTFILE:\"${_manifestfile}\"")

SETUP_PLUGIN(osgearth_model_feature_stencil)
