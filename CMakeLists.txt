# Edit following two lines to set component requirements (see docs)

set(COMPONENT_SRCS "hulp.cpp" "hulp_touch.cpp" "hulp_debug.cpp")
set(COMPONENT_REQUIRES ulp)

set(COMPONENT_PRIV_REQUIRES "soc")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()