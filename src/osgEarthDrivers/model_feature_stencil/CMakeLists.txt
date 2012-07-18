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

IF(MSVC)
	SET(TARGET_LDFLAGS "/MANIFEST:NO")
ENDIF()
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/model_feature_stencil)
SETUP_PLUGIN(osgearth_model_feature_stencil)
