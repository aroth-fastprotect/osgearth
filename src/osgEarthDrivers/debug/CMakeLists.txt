SET(TARGET_SRC DebugTileSource.cpp)
SET(TARGET_PUBLIC_H DebugOptions)
LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthSymbology osgTerrain osgText)
SETUP_PLUGIN(osgearth_debug)
