SET(TARGET_SRC ReaderWriterYahoo.cpp)
SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/yahoo/YahooOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/yahoo)
SETUP_PLUGIN(osgearth_yahoo)
