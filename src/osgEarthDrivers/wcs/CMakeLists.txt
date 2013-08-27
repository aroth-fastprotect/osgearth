
SET(TARGET_SRC 
    ReaderWriterWCS.cpp
    WCS11Source.cpp
)

SET(TARGET_H
    WCS11Source.h
)
SET(TARGET_PUBLIC_H WCSOptions)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/wcs)

SETUP_PLUGIN(osgearth_wcs)
