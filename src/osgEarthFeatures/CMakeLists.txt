IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(LIB_NAME osgEarthFeatures)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})
SET(LIB_PUBLIC_HEADERS
    Annotation  
    BufferFilter
    BuildGeometryFilter  
    Common
    ConvertTypeFilter
    CssUtils
    Feature
    FeatureGridder
    FeatureModelSource
    FeatureSource
    FeatureTileSource
    Filter
    Geometry
    GEOS
	Query
    ResampleFilter
	ScaleFilter
    SLD
    StencilVolumeNode
    Styling
    TransformFilter
)

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
    Annotation.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp
    ConvertTypeFilter.cpp
    CssUtils.cpp
    Feature.cpp
    FeatureGridder.cpp
    FeatureModelSource.cpp
    FeatureSource.cpp
    FeatureTileSource.cpp
    Filter.cpp
    GEOS.cpp
    Geometry.cpp
	Query.cpp
    ResampleFilter.cpp
	ScaleFilter.cpp
    SLD.cpp
    StencilVolumeNode.cpp
    Styling.cpp
    TransformFilter.cpp
)

IF(GEOS_FOUND)
  INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${GEOS_INCLUDE_DIR})
ELSE(GEOS_FOUND)
  INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR})
ENDIF(GEOS_FOUND)


IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
osgEarth
)


LINK_INTERNAL(${LIB_NAME}     
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
   LINK_WITH_VARIABLES(${LIB_NAME} GEOS_LIBRARY)
ENDIF(GEOS_FOUND)

LINK_CORELIB_DEFAULT_VER(${LIB_NAME} ${OSGEARTH_VERSION} ${OSGEARTH_SOVERSION})

INCLUDE(ModuleInstall OPTIONAL)
