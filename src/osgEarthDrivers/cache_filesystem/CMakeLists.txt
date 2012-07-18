
IF (ZLIB_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_ZLIB)
ENDIF(ZLIB_FOUND)

SET(TARGET_PUBLIC_H
    FileSystemCache
)
SET(TARGET_SRC 
    FileSystemCache.cpp
)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/cache_filesystem)

SETUP_PLUGIN(osgearth_cache_filesystem)
