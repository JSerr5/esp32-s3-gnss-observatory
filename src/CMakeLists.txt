# Componente principal (PlatformIO usa src/ como componente "main" de ESP-IDF)
FILE(GLOB app_sources ${CMAKE_SOURCE_DIR}/src/*.c)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS ".")

# Advertencias estrictas solo para el codigo del proyecto
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wformat=2
    -Wshadow
    -Werror=format
    -Werror=implicit-function-declaration
)
