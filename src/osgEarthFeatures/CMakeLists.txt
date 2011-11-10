IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(TARGET_PUBLIC_H
	AltitudeFilter
    BufferFilter
    BuildGeometryFilter  
    BuildTextFilter
    BuildTextOperator
    CentroidFilter
    Common
    ConvertTypeFilter
    CropFilter
    ExtrudeGeometryFilter
    Feature
    FeatureCursor
    FeatureDisplayLayout
    FeatureGeometryIndex
    FeatureGridder
	FeatureListSource
    FeatureModelGraph
    FeatureModelSource    
    FeatureNode
    FeatureSource
	FeatureSourceNode
	FeatureSourceMeshConsolidator
    FeatureTileSource
    Filter
    FilterContext
    GeometryCompiler
	GeometryUtils
    LabelSource
	OgrUtils
    OptimizerHints
    ResampleFilter
    ScaleFilter
    Session
    ScatterFilter
    Script
    ScriptEngine
    SubstituteModelFilter
    TransformFilter
    VirtualFeatureSource
)

SET(TARGET_SRC
	AltitudeFilter.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp 
    BuildTextFilter.cpp
    BuildTextOperator.cpp
    CentroidFilter.cpp
    ConvertTypeFilter.cpp
    CropFilter.cpp
    ExtrudeGeometryFilter.cpp
    Feature.cpp
    FeatureCursor.cpp
    FeatureDisplayLayout.cpp
    FeatureGeometryIndex.cpp
    FeatureGridder.cpp
	FeatureListSource.cpp
    FeatureModelGraph.cpp
    FeatureModelSource.cpp
    FeatureNode.cpp
    FeatureSource.cpp
	FeatureSourceNode.cpp
	FeatureSourceMeshConsolidator.cpp
    FeatureTileSource.cpp
    Filter.cpp
    FilterContext.cpp
    GeometryCompiler.cpp
	GeometryUtils.cpp
    LabelSource.cpp
    OptimizerHints.cpp
    ResampleFilter.cpp
    ScaleFilter.cpp
    Session.cpp
    ScatterFilter.cpp
    ScriptEngine.cpp
    SubstituteModelFilter.cpp
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
