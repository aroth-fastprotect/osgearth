

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC ReaderWriterTileCache.cpp)

SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/tilecache/TileCacheOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/tilecache)

SETUP_PLUGIN(osgearth_tilecache)
