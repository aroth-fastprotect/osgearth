INCLUDE_DIRECTORIES( ${LIBZIP_INCLUDE_DIR} )

SET(TARGET_SRC    
    ReaderWriterWorldWind.cpp)

SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/worldwind/WorldWindOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/worldwind)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include)
	
SET(TARGET_LIBRARIES_VARS LIBZIP_LIBRARY )

SETUP_PLUGIN(osgearth_worldwind)

