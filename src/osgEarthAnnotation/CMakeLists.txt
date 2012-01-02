IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(TARGET_PUBLIC_H
    AnnotationData
    AnnotationNode
    AnnotationUtils
    CircleNode
    Common
    Decluttering
    Decoration
    EllipseNode
    Export
    FeatureNode
    GeometryNode
    HighlightDecoration
    ImageOverlay
    LabelNode
    LocalizedNode
    OrthoNode
    PlaceNode
    ScaleDecoration
    TrackNode
)

SET(TARGET_SRC
    AnnotationData.cpp
    AnnotationNode.cpp
    AnnotationUtils.cpp
    CircleNode.cpp
    Decluttering.cpp
    Decoration.cpp
    EllipseNode.cpp
    FeatureNode.cpp
    GeometryNode.cpp
    HighlightDecoration.cpp
    ImageOverlay.cpp
    LabelNode.cpp
    LocalizedNode.cpp
    OrthoNode.cpp
    PlaceNode.cpp
    TrackNode.cpp
)

LIST(APPEND TARGET_PUBLIC_H
    Draggers
	FeatureEditing
	ImageOverlayEditor
)
    
LIST(APPEND TARGET_SRC
    Draggers.cpp
	FeatureEditing.cpp
    ImageOverlayEditor.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR})

SET(TARGET_ADDED_LIBRARIES
    osgEarth
    osgEarthFeatures
    osgEarthSymbology
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
    OpenThreads
    )

SETUP_OSG_LIB(osgEarthAnnotation)
