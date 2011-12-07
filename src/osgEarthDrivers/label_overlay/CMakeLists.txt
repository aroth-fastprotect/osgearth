SET(TARGET_SRC OverlayLabelSource.cpp)
#SET(TARGET_H OverlayLabelOptions)
LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthFeatures osgEarthSymbology osgEarthUtil osgText)
SETUP_PLUGIN(osgearth_label_overlay)
