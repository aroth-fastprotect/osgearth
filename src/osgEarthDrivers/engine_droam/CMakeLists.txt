
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    CubeManifold.cpp
    Diamond.cpp
    DRoamNode.cpp
    Manifold.cpp
    MeshManager.cpp
    Plugin.cpp
    AMRGeometry.cpp
)

SET(TARGET_H
    AMRShaders.h
	)
SET(TARGET_PUBLIC_H
    ../../../../include/osgEarthDrivers/engine_droam/Common
    ../../../../include/osgEarthDrivers/engine_droam/CubeManifold
    ../../../../include/osgEarthDrivers/engine_droam/Diamond
    ../../../../include/osgEarthDrivers/engine_droam/DRoamNode
    ../../../../include/osgEarthDrivers/engine_droam/Manifold
    ../../../../include/osgEarthDrivers/engine_droam/MeshManager
    ../../../../include/osgEarthDrivers/engine_droam/AMRGeometry
	)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/engine_droam)

SETUP_PLUGIN(osgearth_engine_droam)

