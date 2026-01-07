cmake_minimum_required(VERSION 3.22)
project(POS-random-walk LANGUAGES C)


set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src/common)

# Zatiaľ len malý test/placeholder program, aby bolo čo linkovať
add_executable(pos-rw-test app/test_main.c)
target_link_libraries(pos-rw-test PRIVATE rw_common)

# Include pre app (aby videl include/common/...)
target_include_directories(pos-rw-test PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(pos-rw-test-server app/test_server.c)
target_link_libraries(pos-rw-test-server PRIVATE rw_common)
target_include_directories(pos-rw-test-server PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(pos-rw-test-client app/test_client.c)
target_link_libraries(pos-rw-test-client PRIVATE rw_common)
target_include_directories(pos-rw-test-client PRIVATE ${PROJECT_SOURCE_DIR}/include)


add_executable(pos-rw-state-server app/test_state_server.c)
target_link_libraries(pos-rw-state-server PRIVATE rw_common)
target_include_directories(pos-rw-state-server PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(pos-rw-state-client app/test_state_client.c)
target_link_libraries(pos-rw-state-client PRIVATE rw_common)
target_include_directories(pos-rw-state-client PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(pos-rw-create-server app/create_server.c)
target_link_libraries(pos-rw-create-server PRIVATE rw_common)
target_include_directories(pos-rw-create-server PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(pos-rw-create-client app/create_client.c)
target_link_libraries(pos-rw-create-client PRIVATE rw_common)
target_include_directories(pos-rw-create-client PRIVATE ${PROJECT_SOURCE_DIR}/include)


add_executable(server app/main_server.c)
target_link_libraries(server PRIVATE rw_common)
target_include_directories(server PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(client app/main_client.c)
target_link_libraries(client PRIVATE rw_common)
target_include_directories(client PRIVATE ${PROJECT_SOURCE_DIR}/include)
