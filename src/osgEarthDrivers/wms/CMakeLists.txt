
SET(TARGET_SRC
    ReaderWriterWMS.cpp
    TileService.cpp)

SET(TARGET_H
    TileService
)
SET(TARGET_PUBLIC_H WMSOptions)

LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthUtil)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/wms)

SETUP_PLUGIN(osgearth_wms)
