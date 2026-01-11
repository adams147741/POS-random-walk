add_library(rw_common STATIC
    socket.c
)

target_include_directories(rw_common PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_options(rw_common PRIVATE -Wall -Wextra -Wpedantic)