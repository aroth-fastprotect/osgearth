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
    Common
    Content
    CssUtils
    ExtrudedSymbol
    Geometry
    GeometryExtrudeSymbolizer
    GeometrySymbolizer
    GeometrySymbol
    GEOS
    GeometryRasterizer
	LineFunctor
    MarkerSymbolizer
    MarkerSymbol
	MeshSubdivider
    ModelSymbolizer
	Query
    Registry
	StencilVolumeNode
    Style
    SLD
    StyleSheet
    Symbol
    SymbolicNode
    Symbolizer
	Text
)

SET(TARGET_SRC
    CssUtils.cpp
    ExtrudedSymbol.cpp
    Geometry.cpp
    GeometryExtrudeSymbolizer.cpp
    GeometrySymbolizer.cpp
    GeometrySymbol.cpp
    GEOS.cpp
    GeometryRasterizer.cpp
    MarkerSymbolizer.cpp
    MarkerSymbol.cpp
    MeshSubdivider.cpp
    ModelSymbolizer.cpp
    Query.cpp
    Registry.cpp
    StencilVolumeNode.cpp
    Style.cpp
    SLD.cpp
    StyleSheet.cpp
    Symbol.cpp
    SymbolicNode.cpp
    Symbolizer.cpp
	Text.cpp
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

