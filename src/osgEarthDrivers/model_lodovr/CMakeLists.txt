SET(TARGET_SRC LODScaleOverrideModelSource.cpp)
SET(TARGET_PUBLIC_H LODScaleOverrideModelOptions)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_lodovr)
