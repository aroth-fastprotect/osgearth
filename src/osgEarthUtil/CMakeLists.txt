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
	EarthManipulator
	ElevationManager
    ExampleResources
	Export
    FeatureManipTool
    FeatureQueryTool
    Formatter
    GeodeticGraticule
    LatLongFormatter
	LineOfSight
    LinearLineOfSight
	MeasureTool
    MGRSFormatter
	MGRSGraticule
	MouseCoordsTool
	ObjectPlacer
	ObjectLocator
    PolyhedralLineOfSight
    RadialLineOfSight
	SkyNode
	SpatialData
	StarData
	TerrainProfile
    TFS
    TFSPackager
    TMS
    TMSPackager
    UTMGraticule
	WFS
	WMS
)
IF(NOT ${OPENSCENEGRAPH_VERSION} VERSION_LESS "3.1.0")
    SET(TARGET_PUBLIC_H ${TARGET_PUBLIC_H}
        ShadowUtils)
ENDIF()

SET(TARGET_SRC
	AnnotationEvents.cpp
	AutoClipPlaneHandler.cpp
	ClampCallback.cpp
    Controls.cpp
	EarthManipulator.cpp
	ElevationManager.cpp
    ExampleResources.cpp
    FeatureManipTool.cpp
    FeatureQueryTool.cpp
    GeodeticGraticule.cpp
    LatLongFormatter.cpp
    LinearLineOfSight.cpp
	MeasureTool.cpp
    MGRSFormatter.cpp
	MGRSGraticule.cpp
	MouseCoordsTool.cpp
	ObjectPlacer.cpp
    ObjectLocator.cpp
    PolyhedralLineOfSight.cpp
    RadialLineOfSight.cpp
    SpatialData.cpp
    SkyNode.cpp
	TerrainProfile.cpp
    TFS.cpp
    TFSPackager.cpp
    TMS.cpp
    TMSPackager.cpp
    UTMGraticule.cpp
	WFS.cpp
    WMS.cpp
)
IF(NOT ${OPENSCENEGRAPH_VERSION} VERSION_LESS "3.1.0")
    SET(TARGET_SRC ${TARGET_SRC} 
        ShadowUtils.cpp)
ENDIF()

LIST(APPEND TARGET_PUBLIC_H
    BrightnessContrastColorFilter
    CMYKColorFilter
    GammaColorFilter
    HSLColorFilter
    RGBColorFilter
    ChromaKeyColorFilter
)

LIST(APPEND TARGET_SRC
    BrightnessContrastColorFilter.cpp
    CMYKColorFilter.cpp
    GammaColorFilter.cpp
    HSLColorFilter.cpp
    RGBColorFilter.cpp
    ChromaKeyColorFilter.cpp
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
	osgShadow
    OpenThreads
    )

SETUP_OSG_LIB(osgEarthUtil)

