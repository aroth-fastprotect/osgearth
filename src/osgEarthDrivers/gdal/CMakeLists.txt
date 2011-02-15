INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )

SET(TARGET_SRC
    ReaderWriterGDAL.cpp
)
SET(TARGET_PUBLIC_H ../../../../include/osgEarthDrivers/gdal/GDALOptions)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/gdal)

SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY EXPAT_LIBRARY)

SETUP_PLUGIN(osgearth_gdal)


# to install public driver includes:
SET(LIB_NAME gdal)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

