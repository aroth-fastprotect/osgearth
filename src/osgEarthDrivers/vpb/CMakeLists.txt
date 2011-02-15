SET(TARGET_SRC
    ReaderWriterVPB.cpp
)

SET(TARGET_PUBLIC_H
    ../../../../include/osgEarthDrivers/vpb/VPBOptions
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/vpb)

SETUP_PLUGIN(osgearth_vpb)
