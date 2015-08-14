
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    CubeManifold.cpp
    Diamond.cpp
    DRoamNode.cpp
    GeodeticManifold.cpp
    Manifold.cpp
    MeshManager.cpp
    Plugin.cpp
    AMRGeometry.cpp
)

SET(TARGET_H
    AMRShaders.h
	)
SET(TARGET_PUBLIC_H
    Common
    CubeManifold
    Diamond
    DRoamNode
    GeodeticManifold
    Manifold
    MeshManager
    AMRGeometry
	)
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/engine_droam)

SETUP_PLUGIN(osgearth_engine_droam)

