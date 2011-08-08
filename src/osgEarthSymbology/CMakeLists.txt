IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(LIB_NAME osgEarthSymbology)

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
    MeshSubdivider
    MarkerSymbol
    PointSymbol
    PolygonSymbol
	Query
	StencilVolumeNode
    Style
    SLD
    Symbol
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
    MeshSubdivider.cpp
    MarkerSymbol.cpp
    PointSymbol.cpp
    PolygonSymbol.cpp
    Query.cpp
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

