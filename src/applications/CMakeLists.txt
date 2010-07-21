#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
    osgEarthFeatures
	osgEarthUtil
    osgEarthSymbology
)

SET(TARGET_DEFAULT_PREFIX "application_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Sample")

IF(BUILD_OSG_EXAMPLES)
	ADD_SUBDIRECTORY(osgearth_simple)
	ADD_SUBDIRECTORY(osgearth_seed)
	ADD_SUBDIRECTORY(osgearth_clouds)
	ADD_SUBDIRECTORY(osgearth_ocean)
	ADD_SUBDIRECTORY(osgearth_tests)
	ADD_SUBDIRECTORY(osgearth_toc)
	ADD_SUBDIRECTORY(osgearth_fadelayers)
	ADD_SUBDIRECTORY(osgearth_elevation)
	ADD_SUBDIRECTORY(osgearth_loadmaps)
	ADD_SUBDIRECTORY(osgearth_earthfile)
	ADD_SUBDIRECTORY(osgearth_features)
	ADD_SUBDIRECTORY(osgearth_symbology)
	ADD_SUBDIRECTORY(osgearth_contour)
ENDIF(BUILD_OSG_EXAMPLES)

ADD_SUBDIRECTORY(osgearth_version)