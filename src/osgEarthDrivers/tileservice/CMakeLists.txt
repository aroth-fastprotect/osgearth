SET(TARGET_SRC
    ReaderWriterTileService.cpp
)
SET(TARGET_PUBLIC_H
    TileServiceOptions
)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/tileservice)

SETUP_PLUGIN(osgearth_tileservice)
