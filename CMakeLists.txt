cmake_minimum_required(VERSION 3.16)
project(raylib_physics_magnet LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
    add_compile_options(/utf-8)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# --- raylib ---------------------------------------------------------------
# システムに入っていれば使い、無ければ GitHub から取得する
# （取得済みのソースを使うなら -DFETCHCONTENT_SOURCE_DIR_RAYLIB=<raylib-src>）
find_package(raylib 5.0 QUIET)
if(NOT raylib_FOUND)
    include(FetchContent)
    set(FETCHCONTENT_QUIET FALSE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        5.5
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(raylib)
endif()

# --- 物理コア（raylib 非依存の静的ライブラリ）-----------------------------
add_library(physics STATIC
    src/phys/collision.cpp
    src/phys/world.cpp
    src/phys/magnet.cpp
    src/phys/electric.cpp
    src/phys/current.cpp)
target_include_directories(physics PUBLIC src)

# --- デモ ------------------------------------------------------------------
add_executable(demo src/main.cpp src/fun_scenes.cpp src/electric_scenes.cpp src/current_scenes.cpp)
target_link_libraries(demo PRIVATE physics raylib)

if(APPLE)
    target_link_libraries(demo PRIVATE
        "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()

# --- ヘッドレステスト -------------------------------------------------------
enable_testing()
add_executable(core_test tests/core_test.cpp)
target_link_libraries(core_test PRIVATE physics)
add_test(NAME core_test COMMAND core_test)

add_executable(magnet_test tests/magnet_test.cpp)
target_link_libraries(magnet_test PRIVATE physics)
add_test(NAME magnet_test COMMAND magnet_test)

add_executable(electric_test tests/electric_test.cpp)
target_link_libraries(electric_test PRIVATE physics)
add_test(NAME electric_test COMMAND electric_test)

add_executable(current_test tests/current_test.cpp)
target_link_libraries(current_test PRIVATE physics)
add_test(NAME current_test COMMAND current_test)

# --- MAGNET FLOW（磁力移動のゲームのプロトタイプ）--------------------------
add_library(flowcore STATIC
    src/flow/flow_world.cpp
    src/flow/flow_stages.cpp
    src/flow/flow_grand.cpp
    src/flow/flow_lab.cpp
    src/flow/flow_records.cpp)
target_link_libraries(flowcore PUBLIC physics)

add_executable(magnet_flow src/flow/flow_main.cpp src/flow/flow_audio.cpp)
target_link_libraries(magnet_flow PRIVATE flowcore raylib)
if(APPLE)
    target_link_libraries(magnet_flow PRIVATE
        "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()

add_executable(flow_test tests/flow_test.cpp)
target_link_libraries(flow_test PRIVATE flowcore)
add_test(NAME flow_test COMMAND flow_test)
