IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthUtil)

SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(LIB_PUBLIC_HEADERS
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

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
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

LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

LINK_INTERNAL(${LIB_NAME}
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

#LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
LINK_CORELIB_DEFAULT_VER(${LIB_NAME} ${OSGEARTH_VERSION} ${OSGEARTH_SOVERSION})

INCLUDE(ModuleInstall OPTIONAL)
