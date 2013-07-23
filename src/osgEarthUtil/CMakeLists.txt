IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(HEADERS_ROOT
    AnnotationEvents
    AutoClipPlaneHandler
    Common
    Controls
    ContourMap
    ClampCallback
    DataScanner
    DetailTexture
    EarthManipulator
    ExampleResources
    Export
    FeatureManipTool
    FeatureQueryTool
    Formatter
    GeodeticGraticule
    HTM
    LatLongFormatter
    LineOfSight
    LinearLineOfSight
    LODBlending
    MeasureTool
    MGRSFormatter
    MGRSGraticule
    MouseCoordsTool
    NormalMap
    ObjectLocator
    PolyhedralLineOfSight
    RadialLineOfSight
    SkyNode
    SpatialData
    StarData
    TerrainProfile
	TileIndex
	TileIndexBuilder
    TFS
    TFSPackager
    TMS
    TMSBackFiller
    TMSPackager
    UTMGraticule
    VerticalScale
    WFS
    WMS
)
IF(NOT ${OPENSCENEGRAPH_VERSION} VERSION_LESS "3.1.0")
    SET(TARGET_PUBLIC_H ${TARGET_PUBLIC_H}
        ShadowUtils)
ENDIF()

SET(SOURCES_ROOT
    AnnotationEvents.cpp
    AutoClipPlaneHandler.cpp
    ClampCallback.cpp
    Controls.cpp
    ContourMap.cpp
    DataScanner.cpp
    DetailTexture.cpp
    EarthManipulator.cpp
    ExampleResources.cpp
    FeatureManipTool.cpp
    FeatureQueryTool.cpp
    GeodeticGraticule.cpp
    HTM.cpp
    LatLongFormatter.cpp
    LinearLineOfSight.cpp
    LODBlending.cpp
    MeasureTool.cpp
    MGRSFormatter.cpp
    MGRSGraticule.cpp
    MouseCoordsTool.cpp
    NormalMap.cpp
    ObjectLocator.cpp
    PolyhedralLineOfSight.cpp
    RadialLineOfSight.cpp
    SpatialData.cpp
    SkyNode.cpp
    TerrainProfile.cpp
	TileIndex.cpp
	TileIndexBuilder.cpp
    TFS.cpp
    TFSPackager.cpp
    TMS.cpp
    TMSBackFiller.cpp
    TMSPackager.cpp
    UTMGraticule.cpp
    VerticalScale.cpp
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
    GLSLColorFilter
    HSLColorFilter
    RGBColorFilter
    ChromaKeyColorFilter
)

LIST(APPEND TARGET_SRC
    BrightnessContrastColorFilter.cpp
    CMYKColorFilter.cpp
    GammaColorFilter.cpp
    GLSLColorFilter.cpp
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

