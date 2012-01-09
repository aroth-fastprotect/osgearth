IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(TARGET_PUBLIC_H
	AnnotationEvents
	AutoClipPlaneHandler
	Common
	Controls
	ClampCallback
    DPLineSegmentIntersector
	EarthManipulator
	ElevationManager
	Export
    Formatters
	Graticule
	LineOfSight
	MeasureTool
	NearFarGroup
	ObjectPlacer
	ObjectLocator
    #OceanSurfaceNode
    Pickers
	SkyNode
	SpatialData
	StarData
    TMS
    TMSPackager
	WFS
	WMS
)

SET(TARGET_SRC
	AnnotationEvents.cpp
	AutoClipPlaneHandler.cpp
	ClampCallback.cpp
    Controls.cpp
    DPLineSegmentIntersector.cpp
	EarthManipulator.cpp
	ElevationManager.cpp
    Formatters.cpp
	Graticule.cpp
	LineOfSight.cpp
	MeasureTool.cpp
	NearFarGroup.cpp
	ObjectPlacer.cpp
    ObjectLocator.cpp
    #OceanSurfaceNode.cpp
    Pickers.cpp
    SpatialData.cpp
    SkyNode.cpp
    TMS.cpp
    TMSPackager.cpp
	WFS.cpp
    WMS.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR})

SET(TARGET_ADDED_LIBRARIES
	osgEarth
	osgEarthFeatures
	osgEarthSymbology
	osgEarthAnnotation
    osg
    osgUtil
    osgSim
    osgTerrain
    osgDB
    osgFX
    osgViewer
    osgWidget
	osgText
	osgGA
	osgManipulator
    OpenThreads
    )

SETUP_OSG_LIB(osgEarthUtil)

