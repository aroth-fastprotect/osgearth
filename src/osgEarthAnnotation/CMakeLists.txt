IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(TARGET_PUBLIC_H
	AnnotationSettings
    AnnotationEditing
    AnnotationData
    AnnotationNode
    AnnotationRegistry
    AnnotationUtils
    CircleNode
    Common
    Decoration
    EllipseNode
    Export
    FeatureNode
    LocalGeometryNode
    HighlightDecoration
    ImageOverlay
    ImageOverlayEditor
    LabelNode
    LocalizedNode
    ModelNode
    OrthoNode
    PlaceNode
	RectangleNode
    ScaleDecoration
    TrackNode
)

SET(TARGET_SRC
    AnnotationEditing.cpp
    AnnotationSettings.cpp
    AnnotationData.cpp
    AnnotationNode.cpp
    AnnotationRegistry.cpp
    AnnotationUtils.cpp
    CircleNode.cpp
    Decoration.cpp
    EllipseNode.cpp
    FeatureNode.cpp
    LocalGeometryNode.cpp
    HighlightDecoration.cpp
    ImageOverlay.cpp
    ImageOverlayEditor.cpp
    LabelNode.cpp
    LocalizedNode.cpp
    RectangleNode.cpp
    ModelNode.cpp
    OrthoNode.cpp
    PlaceNode.cpp
    TrackNode.cpp
)

LIST(APPEND TARGET_PUBLIC_H
	AnnotationEditing
	FeatureEditing
	ImageOverlayEditor
)
    
LIST(APPEND TARGET_SRC
	AnnotationEditing.cpp
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
