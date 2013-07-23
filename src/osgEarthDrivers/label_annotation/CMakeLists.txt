SET(TARGET_SRC AnnotationLabelSource.cpp)
SET(TARGET_PUBLIC_H AnnotationLabelOptions)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthAnnotation)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/label_annotation)
SETUP_PLUGIN(osgearth_label_annotation)
