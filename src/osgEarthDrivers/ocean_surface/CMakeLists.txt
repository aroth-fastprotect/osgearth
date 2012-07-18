
SET(TARGET_SRC ElevationProxyImageLayer.cpp
               OceanCompositor.cpp
               OceanSurfaceContainer.cpp
               ReaderWriterOceanSurface.cpp
)

SET(TARGET_PUBLIC_H OceanSurface)
               
SET(TARGET_H   ElevationProxyImageLayer
               OceanCompositor
               OceanSurface
               OceanSurfaceContainer
               OceanShaders
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/ocean_surface)
SETUP_PLUGIN(osgearth_ocean_surface)
