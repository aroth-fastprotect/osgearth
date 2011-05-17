IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthUtil)

SET(TARGET_PUBLIC_H
	AutoClipPlaneHandler
	Common
	Controls
	ClampCallback
	EarthManipulator
	ElevationManager
	Export
	Graticule
	ObjectPlacer
	ObjectLocator
	OceanSurfaceNode
	SkyNode
	SpatialData
	StarData
	Viewpoint
	WMS
)

SET(TARGET_SRC
	AutoClipPlaneHandler.cpp
	ClampCallback.cpp
    Controls.cpp
	EarthManipulator.cpp
	ElevationManager.cpp
	Graticule.cpp
	ObjectPlacer.cpp
    ObjectLocator.cpp
    OceanSurfaceNode.cpp
    SpatialData.cpp
    SkyNode.cpp
	Viewpoint.cpp
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
    OpenThreads
    )

SETUP_OSG_LIB(osgEarthUtil)

