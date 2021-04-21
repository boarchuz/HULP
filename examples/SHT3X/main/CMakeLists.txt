idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ""
)
# show all logs from this component, otherwise overridden by CONFIG_LOG_DEFAULT_LEVEL_ERROR
target_compile_options(${COMPONENT_LIB} PRIVATE "-D LOG_LOCAL_LEVEL=ESP_LOG_INFO")