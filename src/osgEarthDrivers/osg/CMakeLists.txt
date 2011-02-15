SET(TARGET_PUBLIC_H
    ../../../../include/osgEarthDrivers/osg/OSGOptions
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/osg)

SET(TARGET_SRC 
    OSGTileSource.cpp
)
SETUP_PLUGIN(osgearth_osg)
