SET(TARGET_PUBLIC_H EarthFileSerializer)

SET(TARGET_SRC 
    ReaderWriterOsgEarth.cpp
    EarthFileSerializer1.cpp
    EarthFileSerializer2.cpp)

SET(TARGET_H ${CMAKE_CURRENT_BINARY_DIR}/osgEarthUtilLibraryName.h)

FAST_GET_TARGET_OUTPUT_FILENAME_ONLY(osgEarthUtil LIBNAME_UTIL SHARED_MODULE)
FAST_CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/osgEarthUtilLibraryName.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/osgEarthUtilLibraryName.h)

SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/earth)
#### end var setup  ###
SETUP_PLUGIN(earth)
