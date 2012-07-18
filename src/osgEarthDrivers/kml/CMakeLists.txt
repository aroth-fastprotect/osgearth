FAST_PACKAGE_ZLIB()
FAST_PACKAGE_MINIZIP()

IF(WIN32)
    SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
ENDIF(WIN32)
    
set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} 
    osgEarthFeatures 
    osgEarthSymbology 
    osgEarthUtil
    osgEarthAnnotation
)

LIST(APPEND TARGET_LIBRARIES_VARS ZLIB_LIBRARY MINIZIP_LIBRARY )
INCLUDE_DIRECTORIES( ${ZLIB_INCLUDE_DIR} ${MINIZIP_INCLUDE_DIR} )
ADD_DEFINITIONS(-DOSGEARTH_HAVE_ZLIB -DOSGEARTH_HAVE_MINIZIP )

SET(TARGET_PUBLIC_H
    KML
    KMLOptions
	)

SET(TARGET_H
    KMLReader
    KML_Common
    KML_Container
    KML_Document
    KML_Feature
    KML_Folder
    KML_Geometry
    KML_GroundOverlay
    KML_IconStyle
    KML_LabelStyle
    KML_LinearRing
    KML_LineString
    KML_LineStyle
    KML_Model
    KML_MultiGeometry
    KML_NetworkLinkControl
    KML_Object
    KML_Overlay
    KML_PhotoOverlay
    KML_Point
    KML_Polygon
    KML_PolyStyle
    KML_Root
    KML_Schema
    KML_ScreenOverlay
    KML_Style
    KML_StyleMap
    KML_StyleSelector
    KMZArchive
)

SET(TARGET_SRC
    ReaderWriterKML.cpp
    KMLReader.cpp
    
    KML_Document.cpp
    KML_Feature.cpp
    KML_Folder.cpp
    KML_Geometry.cpp
    KML_GroundOverlay.cpp
    KML_IconStyle.cpp
    KML_LabelStyle.cpp
    KML_LinearRing.cpp
    KML_LineString.cpp
    KML_LineStyle.cpp
    KML_Model.cpp
    KML_MultiGeometry.cpp
    KML_NetworkLink.cpp
    KML_NetworkLinkControl.cpp
    KML_Object.cpp
    KML_Overlay.cpp
    KML_PhotoOverlay.cpp
    KML_Placemark.cpp
    KML_Point.cpp
    KML_Polygon.cpp
    KML_PolyStyle.cpp
    KML_Root.cpp
    KML_Schema.cpp
    KML_ScreenOverlay.cpp
    KML_Style.cpp
    KML_StyleMap.cpp
    KMZArchive.cpp
)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/kml)   
SETUP_PLUGIN(kml)
