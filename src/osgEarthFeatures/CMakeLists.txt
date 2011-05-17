IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(LIB_NAME osgEarthFeatures)

SET(TARGET_PUBLIC_H
	Annotation  
	BufferFilter
	BuildGeometryFilter  
	Common
	ConvertTypeFilter
	Feature
	FeatureGridder
	FeatureModelSource
	FeatureSource
	FeatureSymbolizer
	FeatureTileSource
	Filter
	ResampleFilter
	ScaleFilter
	BuildTextOperator
	TransformFilter
	VirtualFeatureSource
)

SET(TARGET_SRC
    Annotation.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp
    ConvertTypeFilter.cpp
    Feature.cpp
    FeatureGridder.cpp
    FeatureModelSource.cpp
    FeatureSource.cpp
    FeatureSymbolizer.cpp
    FeatureTileSource.cpp
    Filter.cpp
    ResampleFilter.cpp
	ScaleFilter.cpp
	BuildTextOperator.cpp
    TransformFilter.cpp
    VirtualFeatureSource.cpp
)

IF(GEOS_FOUND)
  INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${GEOS_INCLUDE_DIR})
ELSE(GEOS_FOUND)
  INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR})
ENDIF(GEOS_FOUND)

SET(TARGET_ADDED_LIBRARIES
	osgEarth
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
IF(GEOS_FOUND)
	SET(TARGET_LIBRARIES_VARS GEOS_LIBRARY)
ENDIF(GEOS_FOUND)
SETUP_OSG_LIB(osgEarthFeatures)
