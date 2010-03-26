IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHSYMBOLOGY_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)


SET(LIB_NAME osgEarthSymbology)

SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(LIB_PUBLIC_HEADERS
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

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
#  .cpp files go here
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

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
    osgEarth
    osgEarthFeatures
)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
