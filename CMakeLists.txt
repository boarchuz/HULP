set(srcs 
    "src/hulp.c"
    "src/hulp_touch.c"
    "src/hulp_uart.c"
    "src/hulp_regwr.c"
    "src/hulp_debug.c"
)

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