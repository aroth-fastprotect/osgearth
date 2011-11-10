IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(TARGET_PUBLIC_H
    AnnotationData
    CircleNode
    Common
    Decluttering
    EllipseNode
    Export
    GeometryNode
    ImageOverlay
    LabelNode
    LocalizedNode
    PlaceNode
    TrackNode
)

SET(TARGET_SRC
    AnnotationData.cpp
    CircleNode.cpp
    Decluttering.cpp
    EllipseNode.cpp
    GeometryNode.cpp
    ImageOverlay.cpp
    LabelNode.cpp
    LocalizedNode.cpp
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
