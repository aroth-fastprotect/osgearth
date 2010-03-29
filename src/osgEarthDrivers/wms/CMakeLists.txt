
SET(TARGET_SRC
    Capabilities.cpp
    ReaderWriterWMS.cpp
    TileService.cpp)

SET(TARGET_H
    Capabilities
    TileService
)
SET(TARGET_PUBLIC_H WMSOptions)
SETUP_PLUGIN(osgearth_wms)
