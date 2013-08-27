
LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthSymbology osgTerrain)

SET(TARGET_SRC
    TilePagedLOD.cpp
    KeyNodeFactory.cpp
    MPGeometry.cpp
    MPTerrainEngineNode.cpp
    MPTerrainEngineDriver.cpp
    SerialKeyNodeFactory.cpp
    TerrainNode.cpp
    TileGroup.cpp
    TileModel.cpp
    TileModelCompiler.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    TileModelFactory.cpp
)
SET(TARGET_PUBLIC_H
    MPTerrainEngineOptions
)

SET(TARGET_H
    Common
    TilePagedLOD
    DynamicLODScaleCallback
    FileLocationCallback
    KeyNodeFactory
    MPGeometry
    MPTerrainEngineNode
    QuickReleaseGLObjects
    SerialKeyNodeFactory
    TerrainNode
    TileGroup
    TileModel
    TileModelCompiler
    TileNode
    TileNodeRegistry
    TileModelFactory
)
IF(WIN32)
	LIST(APPEND TARGET_ADDED_LIBRARIES opengl32.lib)
ELSEIF(UNIX)
    LIST(APPEND TARGET_ADDED_LIBRARIES OPENGL_gl_LIBRARY)
ENDIF()

SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/engine_mp)
SETUP_PLUGIN(osgearth_engine_mp)
