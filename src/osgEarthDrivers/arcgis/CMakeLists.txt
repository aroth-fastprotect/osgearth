
SET(TARGET_SRC
    ReaderWriterArcGIS.cpp
    MapService.cpp
)

# headers to show in IDE
SET(TARGET_H
    Extent.h
    MapService.h
)

SET(TARGET_PUBLIC_H ArcGISOptions)

SETUP_PLUGIN(osgearth_arcgis)
