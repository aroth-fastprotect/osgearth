IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(TARGET_PUBLIC_H
	AutoClipPlaneHandler
	Common
	Controls
	ClampCallback
	EarthManipulator
	ElevationManager
	Export
    Formatters
	Graticule
	MeasureTool
	NearFarGroup
	ObjectPlacer
	ObjectLocator
    #OceanSurfaceNode
	SkyNode
	SpatialData
	StarData
	WFS
	WMS
)

SET(TARGET_SRC
	AutoClipPlaneHandler.cpp
	ClampCallback.cpp
    Controls.cpp
	EarthManipulator.cpp
	ElevationManager.cpp
    Formatters.cpp
	Graticule.cpp
	MeasureTool.cpp
	NearFarGroup.cpp
	ObjectPlacer.cpp
    ObjectLocator.cpp
    #OceanSurfaceNode.cpp
    SpatialData.cpp
    SkyNode.cpp
	WFS.cpp
    WMS.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR})

SET(TARGET_ADDED_LIBRARIES
	osgEarth
	osgEarthFeatures
	osgEarthSymbology
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

