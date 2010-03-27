IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(LIB_NAME osgEarthFeatures)

SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(TARGET_PUBLIC_H
	${HEADER_PATH}/Annotation  
	${HEADER_PATH}/BufferFilter
	${HEADER_PATH}/BuildGeometryFilter  
	${HEADER_PATH}/Common
	${HEADER_PATH}/ConvertTypeFilter
	${HEADER_PATH}/CssUtils
	${HEADER_PATH}/DriverOptions
	${HEADER_PATH}/Feature
	${HEADER_PATH}/FeatureGridder
	${HEADER_PATH}/FeatureModelSource
	${HEADER_PATH}/FeatureSource
	${HEADER_PATH}/FeatureTileSource
	${HEADER_PATH}/Filter
	${HEADER_PATH}/Geometry
	${HEADER_PATH}/GEOS
	${HEADER_PATH}/Query
	${HEADER_PATH}/ResampleFilter
	${HEADER_PATH}/ScaleFilter
	${HEADER_PATH}/SLD
	${HEADER_PATH}/StencilVolumeNode
	${HEADER_PATH}/Styling
	${HEADER_PATH}/TransformFilter
)

SET(TARGET_SRC
    Annotation.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp
    ConvertTypeFilter.cpp
    CssUtils.cpp
    DriverOptions.cpp
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
	SET(TARGET_LIBRARIES_VARS GEOS_LIBRARY)
ENDIF(GEOS_FOUND)
SETUP_OSG_LIB(osgEarthFeatures)
