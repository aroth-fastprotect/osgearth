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
	ImageOverlay	
	MeasureTool
	ObjectPlacer
	ObjectLocator
	OceanSurfaceNode
	SkyNode
	SpatialData
	StarData
	Viewpoint
	WFS
	WMS
)

SET(TARGET_SRC
	AutoClipPlaneHandler.cpp
	ClampCallback.cpp
    Controls.cpp
	EarthManipulator.cpp
	ElevationManager.cpp
	Graticule.cpp
	ImageOverlay.cpp
	MeasureTool.cpp
	ObjectPlacer.cpp
    ObjectLocator.cpp
    OceanSurfaceNode.cpp
    SpatialData.cpp
    SkyNode.cpp
	Viewpoint.cpp
	WFS.cpp
    WMS.cpp
)

IF(NOT (${OPENSCENEGRAPH_VERSION} VERSION_LESS "2.9.6"))
SET(TARGET_PUBLIC_H ${TARGET_PUBLIC_H}
    Draggers
	FeatureEditing
	ImageOverlayEditor
	)
	
SET(TARGET_SRC ${TARGET_SRC} 
    Draggers.cpp
	FeatureEditing.cpp
    ImageOverlayEditor.cpp
    )
ENDIF()

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

