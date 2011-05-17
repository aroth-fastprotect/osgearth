
SET(TARGET_SRC
    Capabilities.cpp
    ReaderWriterWMS.cpp
    TileService.cpp)

SET(TARGET_H
    Capabilities
    TileService
)
SET(TARGET_PUBLIC_H WMSOptions)

LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthUtil)
SETUP_PLUGIN(osgearth_wms)
