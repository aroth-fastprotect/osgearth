IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

# Header files that need moc'd
SET(TARGET_PUBLIC_MOC_H
	AnnotationDialogs
    AnnotationListWidget
    AnnotationToolbar
    CollapsiblePairWidget
    DataManager
    LayerManagerWidget
    LOSControlWidget
    LOSCreationDialog
    MapCatalogWidget
    TerrainProfileGraph
    TerrainProfileWidget
    ViewerWidget
)

# Qt resource files
SET(TARGET_QRC
    images.qrc
)

# Qt UI files
SET(TARGET_UI
    ui/LOSCreationDialog.ui
)


SET(TARGET_PUBLIC_H
#   header files go here
    Actions
    AnnotationDialogs
    AnnotationListWidget
    AnnotationToolbar
    CollapsiblePairWidget
    Common
    DataManager
    GuiActions
    LayerManagerWidget
    LOSControlWidget
    LOSCreationDialog
    MapCatalogWidget
    MultiViewerWidget
    TerrainProfileGraph
    TerrainProfileWidget
    ViewWidget
    ViewerWidget
)

SET(TARGET_SRC
	AnnotationDialogs.cpp
    AnnotationListWidget.cpp
    AnnotationToolbar.cpp
    CollapsiblePairWidget.cpp
    DataManager.cpp
    LayerManagerWidget.cpp
    LOSControlWidget.cpp
    LOSCreationDialog.cpp
    MapCatalogWidget.cpp
    MultiViewerWidget.cpp
    TerrainProfileGraph.cpp
    TerrainProfileWidget.cpp
    ViewWidget.cpp
    ViewerWidget.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

SET(TARGET_ADDED_LIBRARIES
    osgEarth
    osgEarthAnnotation
    osgEarthSymbology
    osgEarthFeatures
    osgEarthUtil
    osg
    osgWidget
    osgUtil
    osgSim
    osgTerrain
    osgDB
	osgFX
	osgViewer
	osgText
	osgGA
	osgQt
    OpenThreads
    )

SET(TARGET_LIBRARIES_VARS	
    QT_QTCORE_LIBRARY
    QT_QTGUI_LIBRARY
    QT_QTOPENGL_LIBRARY
	)

SETUP_OSG_LIB(osgEarthQt NO_MOC_INCLUDE)
