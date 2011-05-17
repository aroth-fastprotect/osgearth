SET(TARGET_SRC SimpleModelSource.cpp)
SET(TARGET_PUBLIC_H SimpleModelOptions)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_simple)
