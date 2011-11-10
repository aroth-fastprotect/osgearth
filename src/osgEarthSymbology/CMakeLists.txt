IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(TARGET_PUBLIC_H
#   header files go here
	AGG.h
    AltitudeSymbol
    Common
    Color
    CssUtils
    Expression
    ExtrusionSymbol
    Geometry
    GeometryFactory
    GEOS
    GeometryRasterizer
	LineFunctor
    LineSymbol
    MarkerResource
    MarkerSymbol
    MeshConsolidator
    MeshSubdivider
    PointSymbol
    PolygonSymbol
	Query
    Resource
    ResourceCache
    ResourceLibrary
    Skins
	StencilVolumeNode
    Style
    SLD
    Symbol
    Tags
    TextSymbol
)

SET(TARGET_SRC
    AltitudeSymbol.cpp
    Color.cpp
    CssUtils.cpp
    Expression.cpp
    ExtrusionSymbol.cpp
    Geometry.cpp
    GeometryFactory.cpp
    GEOS.cpp
    GeometryRasterizer.cpp
    LineSymbol.cpp
    MarkerResource.cpp
    MarkerSymbol.cpp
    MeshConsolidator.cpp
    MeshSubdivider.cpp
    PointSymbol.cpp
    PolygonSymbol.cpp
    Query.cpp
    Resource.cpp
    ResourceCache.cpp
    ResourceLibrary.cpp
    Skins.cpp
    StencilVolumeNode.cpp
    Style.cpp
    SLD.cpp
    Symbol.cpp
    TextSymbol.cpp
)


SET(TARGET_ADDED_LIBRARIES
    osgEarth
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

IF(GEOS_FOUND)
	LIST(APPEND TARGET_ADDED_LIBRARIES GEOS_LIBRARY)
ENDIF(GEOS_FOUND)

SETUP_OSG_LIB(osgEarthSymbology)

