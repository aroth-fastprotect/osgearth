IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthUtil)

SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(TARGET_PUBLIC_H
	${HEADER_PATH}/AutoClipPlaneHandler
	${HEADER_PATH}/Common
	${HEADER_PATH}/ClampCallback
	${HEADER_PATH}/EarthManipulator
	${HEADER_PATH}/ElevationFadeCallback
	${HEADER_PATH}/ElevationManager
	${HEADER_PATH}/Export
	${HEADER_PATH}/FadeLayerNode
	${HEADER_PATH}/Graticule
	${HEADER_PATH}/ObjectPlacer
	${HEADER_PATH}/Viewpoint
)

SET(TARGET_SRC
	AutoClipPlaneHandler.cpp
	ClampCallback.cpp
	EarthManipulator.cpp
	ElevationFadeCallback.cpp
	ElevationManager.cpp
	FadeLayerNode.cpp
	Graticule.cpp
	ObjectPlacer.cpp
	Viewpoint.cpp
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
	osgText
	osgGA
    OpenThreads
    )

SETUP_OSG_LIB(osgEarthUtil)

