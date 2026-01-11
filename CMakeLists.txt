cmake_minimum_required(VERSION 3.22)
project(POS-random-walk LANGUAGES C)


set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src/common)

add_executable(server app/main_server.c)
target_link_libraries(server PRIVATE rw_common)
target_include_directories(server PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(client app/main_client.c)
target_link_libraries(client PRIVATE rw_common)
target_include_directories(client PRIVATE ${PROJECT_SOURCE_DIR}/include)
