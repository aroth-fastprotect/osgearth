
SET(TARGET_SRC
    ReaderWriterArcGIS.cpp
    MapService.cpp
)

# headers to show in IDE
SET(TARGET_H
    Extent.h
    MapService.h
)

SET(TARGET_PUBLIC_H 
	../../../../include/osgEarthDrivers/arcgis/ArcGISOptions
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/arcgis)

SETUP_PLUGIN(osgearth_arcgis)
