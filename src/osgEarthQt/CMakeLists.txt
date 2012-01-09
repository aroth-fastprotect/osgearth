IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

# Header files that need moc'd
SET(TARGET_PUBLIC_MOC_H
    AnnotationListWidget
    DataManager
    LayerManagerWidget
    MapCatalogWidget
)

# Qt resource files
SET(TARGET_QRC
    images.qrc
)

SET(TARGET_PUBLIC_H
#   header files go here
    Actions
    Common
    CompositeViewerWidget
    GuiActions
    ViewerWidget
)

SET(TARGET_SRC
    AnnotationListWidget.cpp
    CompositeViewerWidget.cpp
    DataManager.cpp
    LayerManagerWidget.cpp
    MapCatalogWidget.cpp
    ViewerWidget.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${QT_INCLUDES})

SET(TARGET_ADDED_LIBRARIES
    osgEarth
    osgEarthAnnotation
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

SETUP_OSG_LIB(osgEarthQt)
