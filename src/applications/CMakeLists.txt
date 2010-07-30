#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
    osgEarthFeatures
	osgEarthUtil
    osgEarthSymbology
)
# OSG Examples
OPTION(BUILD_OSGEARTH_EXAMPLES "Enable to build osgEarth Examples" OFF)

IF(BUILD_OSGEARTH_EXAMPLES)

	SET(TARGET_DEFAULT_PREFIX "application_")
	SET(TARGET_DEFAULT_LABEL_PREFIX "Sample")

	ADD_SUBDIRECTORY(osgearth_memory)
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
ENDIF(BUILD_OSGEARTH_EXAMPLES)


SET(TARGET_DEFAULT_PREFIX "")
SET(TARGET_DEFAULT_LABEL_PREFIX "Applications")

ADD_SUBDIRECTORY(osgearth_version)
