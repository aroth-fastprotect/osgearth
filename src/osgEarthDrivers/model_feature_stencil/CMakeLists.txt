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
	SET(TARGET_LDFLAGS "/MANIFESTFILE:\"${CMAKE_CURRENT_BINARY_DIR}\\\$(TargetFileName).manifest\"")
ENDIF()

SETUP_PLUGIN(osgearth_model_feature_stencil)
