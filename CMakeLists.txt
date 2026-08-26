cmake_minimum_required(VERSION 3.22)

project(gnss_data_simulator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(cmake/CompilerWarnings.cmake)

set(GNSS_SIM_RTKLIB_COMMIT "47bb6cf9935b510109d16f46baa35c648e56f1b5" CACHE STRING
    "Pinned RTKLIB commit used by this simulator build")

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB/src/rtklib.h")
    message(FATAL_ERROR
        "RTKLIB submodule is missing. Run: git submodule update --init --recursive")
endif()

add_library(rtklib_pinned INTERFACE)
add_library(gnss_sim::rtklib ALIAS rtklib_pinned)
target_include_directories(rtklib_pinned SYSTEM INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB/src")
target_compile_definitions(rtklib_pinned INTERFACE
    GNSS_SIM_RTKLIB_COMMIT="${GNSS_SIM_RTKLIB_COMMIT}")

add_library(gnss_sim_core STATIC
    src/core/simulator.cpp
)
add_library(gnss_sim::core ALIAS gnss_sim_core)
target_include_directories(gnss_sim_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(gnss_sim_core PUBLIC gnss_sim::rtklib)
gnss_sim_set_project_warnings(gnss_sim_core)

add_executable(gnss-data-simulator
    src/app/main.cpp
)
target_link_libraries(gnss-data-simulator PRIVATE gnss_sim::core)
gnss_sim_set_project_warnings(gnss-data-simulator)

if(BUILD_TESTING)
    include(FetchContent)

    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.16.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(googletest)

    add_executable(gnss_sim_unit_tests
        tests/unit/test_sim_time.cpp
    )
    target_link_libraries(gnss_sim_unit_tests PRIVATE
        gnss_sim::core
        GTest::gtest_main
    )
    gnss_sim_set_project_warnings(gnss_sim_unit_tests)

    include(GoogleTest)
    gtest_discover_tests(gnss_sim_unit_tests)
endif()
