
INCLUDE_DIRECTORIES( ${V8_INCLUDE_DIR} )

SET(TARGET_SRC
    JavascriptEngineV8Factory.cpp
    JavascriptEngineV8.cpp
    JSWrappers.cpp
)

SET(TARGET_PUBLIC_H
    JavascriptEngineV8
    JSWrappers
    V8Util
)
LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthFeatures)
LIST(APPEND TARGET_LIBRARIES_VARS V8_LIBRARY)

SETUP_PLUGIN(osgearth_scriptengine_javascript)
