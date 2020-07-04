set(COMPONENT_SRCS "src/hulp.cpp" "src/hulp_touch.cpp" "src/hulp_debug.cpp")
set(COMPONENT_REQUIRES ulp)

set(COMPONENT_PRIV_REQUIRES "soc")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()