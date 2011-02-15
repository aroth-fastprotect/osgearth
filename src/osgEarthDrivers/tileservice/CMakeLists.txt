SET(TARGET_SRC
    ReaderWriterTileService.cpp
)
SET(TARGET_PUBLIC_H
    ../../../../include/osgEarthDrivers/tileservice/TileServiceOptions
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/tileservice)

SETUP_PLUGIN(osgearth_tileservice)
