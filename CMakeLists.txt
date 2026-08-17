cmake_minimum_required(VERSION 3.20)
project(hypxrcompose LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

option(HYPXRCOMPOSE_TESTS "Build the gtest suite" ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(deps REQUIRED IMPORTED_TARGET
    egl
    glesv2
)

# nlohmann_json is header-only; prefer the packaged config, fall back to the
# bare header being on the include path (Arch installs it at /usr/include).
find_package(nlohmann_json QUIET)

# Everything except argument parsing lives in the core library so the test
# binary can drive validate/synth/render in-process rather than through a shell.
add_library(hypxrcompose_core STATIC
    src/Log.cpp
    src/Math.cpp
    src/Timeline.cpp
    src/Stabilize.cpp
    src/Process.cpp
    src/Ffmpeg.cpp
)
target_include_directories(hypxrcompose_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(hypxrcompose_core PUBLIC PkgConfig::deps)
if(nlohmann_json_FOUND)
    target_link_libraries(hypxrcompose_core PUBLIC nlohmann_json::nlohmann_json)
endif()
target_compile_options(hypxrcompose_core PRIVATE -Wall -Wextra)

