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
    src/Bundle.cpp
    src/Validate.cpp
    src/ComposeGL.cpp
    src/Render.cpp
    src/SynthScene.cpp
    src/Synth.cpp
)
target_include_directories(hypxrcompose_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(hypxrcompose_core PUBLIC PkgConfig::deps)
if(nlohmann_json_FOUND)
    target_link_libraries(hypxrcompose_core PUBLIC nlohmann_json::nlohmann_json)
endif()
target_compile_options(hypxrcompose_core PRIVATE -Wall -Wextra)

add_executable(hypxrcompose src/main.cpp)
target_link_libraries(hypxrcompose PRIVATE hypxrcompose_core)
target_compile_options(hypxrcompose PRIVATE -Wall -Wextra)

install(TARGETS hypxrcompose RUNTIME DESTINATION bin)

if(HYPXRCOMPOSE_TESTS)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        enable_testing()
        add_executable(hypxrcompose_tests
            tests/test_math.cpp
            tests/test_timeline.cpp
            tests/test_stabilize.cpp
            tests/test_validate.cpp
            tests/test_endtoend.cpp
            tests/Harness.cpp
        )
        target_include_directories(hypxrcompose_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(hypxrcompose_tests PRIVATE hypxrcompose_core GTest::gtest GTest::gtest_main)
        target_compile_options(hypxrcompose_tests PRIVATE -Wall -Wextra)
        # A segmented render spawns copies of the compositor. Left to itself it
        # spawns /proc/self/exe, which inside the test binary is the test binary;
        # the suite therefore has to be told where the real one is, and to wait
        # for it to be built.
        target_compile_definitions(hypxrcompose_tests PRIVATE HYPXRCOMPOSE_BINARY="$<TARGET_FILE:hypxrcompose>")
        # The suite writes synthetic bundles and frame dumps, and a run leaves
        # hundreds of megabytes behind. Rooting that inside the build directory
        # means `rm -rf build` reclaims it, instead of it accumulating in /tmp
        # once per process id until the filesystem fills - which it did.
        target_compile_definitions(hypxrcompose_tests PRIVATE HYPXRCOMPOSE_TEST_SCRATCH_ROOT="${CMAKE_CURRENT_BINARY_DIR}/test-scratch")
        add_dependencies(hypxrcompose_tests hypxrcompose)
        add_test(NAME hypxrcompose_tests COMMAND hypxrcompose_tests)
    else()
        message(STATUS "GTest not found - skipping the hypxrcompose test suite")
    endif()
endif()
