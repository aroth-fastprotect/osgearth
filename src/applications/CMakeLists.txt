#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
    osgEarthFeatures
	osgEarthUtil
    osgEarthSymbology
    osgEarthAnnotation
)
# OSG Examples
OPTION(BUILD_OSGEARTH_EXAMPLES "Enable to build osgEarth Examples" OFF)

IF(BUILD_OSGEARTH_EXAMPLES)

	SET(TARGET_DEFAULT_PREFIX "application_")
	SET(TARGET_DEFAULT_LABEL_PREFIX "Sample")

	ADD_SUBDIRECTORY(osgearth_clamp)
	ADD_SUBDIRECTORY(osgearth_manip)
	ADD_SUBDIRECTORY(osgearth_seed)
    ADD_SUBDIRECTORY(osgearth_composite)
	ADD_SUBDIRECTORY(osgearth_clouds)
	ADD_SUBDIRECTORY(osgearth_toc)
	ADD_SUBDIRECTORY(osgearth_elevation)
	ADD_SUBDIRECTORY(osgearth_features)
	ADD_SUBDIRECTORY(osgearth_featureinfo)
	ADD_SUBDIRECTORY(osgearth_los)
	ADD_SUBDIRECTORY(osgearth_map)
	ADD_SUBDIRECTORY(osgearth_annotation)
	ADD_SUBDIRECTORY(osgearth_tracks)
	ADD_SUBDIRECTORY(osgearth_featureeditor)
	ADD_SUBDIRECTORY(osgearth_measure)
	ADD_SUBDIRECTORY(osgearth_controls)
	ADD_SUBDIRECTORY(osgearth_shadercomp)
	ADD_SUBDIRECTORY(osgearth_tilesource)
	ADD_SUBDIRECTORY(osgearth_imageoverlay)
	ADD_SUBDIRECTORY(osgearth_city)
	ADD_SUBDIRECTORY(osgearth_graticule)

	IF (QT4_FOUND AND NOT ANDROID AND OSGEARTH_USE_QT)
	    ADD_SUBDIRECTORY(osgearth_qt)
	ENDIF()

ENDIF(BUILD_OSGEARTH_EXAMPLES)

SET(TARGET_DEFAULT_PREFIX "")
SET(TARGET_DEFAULT_LABEL_PREFIX "Applications")
# always build osgearth_version
ADD_SUBDIRECTORY(osgearth_version)
ADD_SUBDIRECTORY(osgearth_package)
ADD_SUBDIRECTORY(osgearth_viewer)

