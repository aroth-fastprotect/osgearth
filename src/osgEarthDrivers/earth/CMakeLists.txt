SET(TARGET_PUBLIC_H EarthFileSerializer)

SET(TARGET_SRC 
    ReaderWriterOsgEarth.cpp
    EarthFileSerializer1.cpp
    EarthFileSerializer2.cpp)

SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/earth)
#### end var setup  ###
SETUP_PLUGIN(earth)
