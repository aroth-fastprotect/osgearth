SET(TARGET_SRC
        QSC.cpp
        Euler.cpp
        Patch.cpp
        PatchGroup.cpp
        PatchSet.cpp
        Projected.cpp
        Geographic.cpp
        GeoPatch.cpp
        SeamlessEngineNode.cpp
        SeamlessPlugin.cpp
)

SET(TARGET_PUBLIC_H 
        QSC
        Euler
        Patch
        PatchInfo
        PatchGroup
        PatchSet
        Projected
        Geographic
        GeoPatch
        SeamlessEngineNode
        SeamlessOptions
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR})

SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/engine_seamless)
# to install public driver includes:
SETUP_PLUGIN(osgearth_engine_seamless)
