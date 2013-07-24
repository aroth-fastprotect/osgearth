INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

SET(TARGET_UI
    ExportDialog.ui
)

SET(TARGET_PUBLIC_MOC_H
    PackageQtMainWindow
    ExportDialog
    WaitDialog
)

set(TARGET_QRC
    images.qrc
)

SET(TARGET_H
    PackageQtMainWindow
    ExportDialog
    WaitDialog
    SceneController.h
    TMSExporter.h
)

SET(TARGET_SRC
    ExportDialog.cpp
    SceneController.cpp
    TMSExporter.cpp
    WaitDialog.cpp
    package_qt.cpp
)

SET(TARGET_ADDED_LIBRARIES
	osg
	osgDB
	osgGA
	osgViewer
	osgTerrain
	osgQt
	OpenThreads
    osgEarthQt
)

SET(TARGET_LIBRARIES_VARS	
    QT_QTCORE_LIBRARY
    QT_QTGUI_LIBRARY
    QT_QTOPENGL_LIBRARY
	)

#### end var setup  ###
SETUP_APPLICATION(osgearth_package_qt NO_MOC_INCLUDE)
