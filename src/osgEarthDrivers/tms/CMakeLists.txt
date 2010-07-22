SET(TARGET_SRC
  ReaderWriterTMS.cpp
)
SET(TARGET_PUBLIC_H
  ../../../../include/osgEarthDrivers/tms/TMSOptions
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/tms)

SETUP_PLUGIN(osgearth_tms)
