IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)


SET(LIB_NAME osgEarthSymbology)

SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(TARGET_PUBLIC_H
#   header files go here
    ${HEADER_PATH}/Common
    ${HEADER_PATH}/CssUtils
    ${HEADER_PATH}/ExtrudedSymbol
    ${HEADER_PATH}/Geometry
    ${HEADER_PATH}/GeometryExtrudeSymbolizer
    ${HEADER_PATH}/GeometryInput
    ${HEADER_PATH}/GeometrySymbolizer
    ${HEADER_PATH}/GeometrySymbol
    ${HEADER_PATH}/MarkerSymbolizer
    ${HEADER_PATH}/MarkerSymbol
    ${HEADER_PATH}/ModelSymbolizer
	${HEADER_PATH}/Query
    ${HEADER_PATH}/Registry
	${HEADER_PATH}/StencilVolumeNode
    ${HEADER_PATH}/Style
    ${HEADER_PATH}/SLD
    ${HEADER_PATH}/StyleSheet
    ${HEADER_PATH}/Symbol
    ${HEADER_PATH}/SymbolicNode
    ${HEADER_PATH}/Symbolizer
)

SET(TARGET_SRC
    CssUtils.cpp
    ExtrudedSymbol.cpp
    Geometry.cpp
    GeometryExtrudeSymbolizer.cpp
    GeometryInput.cpp
    GeometrySymbolizer.cpp
    GeometrySymbol.cpp
    MarkerSymbolizer.cpp
    MarkerSymbol.cpp
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
)


SET(TARGET_ADDED_LIBRARIES
    osgEarth
    osgEarthFeatures
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

SETUP_OSG_LIB(osgEarthSymbology)

