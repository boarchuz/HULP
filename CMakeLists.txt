set(srcs 
    "src/hulp.cpp"
    "src/hulp_touch.cpp"
    "src/hulp_debug.cpp")

set(include_dirs 
    "src"
)

set(requires
    ulp
    driver
)

set(priv_requires
    soc
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES "${requires}"
    PRIV_REQUIRES "${priv_requires}"
)