
SET(TARGET_SRC
    Capabilities.cpp
    ReaderWriterWMS.cpp
    TileService.cpp)

SET(TARGET_H
    Capabilities
    TileService
)
SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/wms/WMSOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/wms)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include)

LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthUtil)
SETUP_PLUGIN(osgearth_wms)
