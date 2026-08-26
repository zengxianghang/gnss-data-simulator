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

set(GNSS_SIM_RTKLIB_COMMIT "07e813b72c8667350c4e80293cb6679c519ef1a6" CACHE STRING
    "Pinned RTKLIB commit used by this simulator build")
set(GNSS_SIM_CJSON_COMMIT "acc76239bee01d8e9c858ae2cab296704e52d916" CACHE STRING
    "Pinned cJSON commit used by this simulator build")

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB/src/rtklib.h")
    message(FATAL_ERROR
        "RTKLIB submodule is missing. Run: git submodule update --init --recursive")
endif()
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/cJSON/cJSON.h")
    message(FATAL_ERROR
        "cJSON submodule is missing. Run: git submodule update --init --recursive")
endif()

find_package(Threads REQUIRED)

add_library(rtklib_pinned STATIC
    third_party/RTKLIB/src/ephemeris.c
    third_party/RTKLIB/src/preceph.c
    third_party/RTKLIB/src/qzslex.c
    third_party/RTKLIB/src/rinex.c
    third_party/RTKLIB/src/rtcm.c
    third_party/RTKLIB/src/rtcm2.c
    third_party/RTKLIB/src/rtcm3.c
    third_party/RTKLIB/src/rtcm3e.c
    third_party/RTKLIB/src/rtkcmn.c
    third_party/RTKLIB/src/rtklib_obs_ext.c
    third_party/RTKLIB/src/sbas.c
)
add_library(gnss_sim::rtklib ALIAS rtklib_pinned)
target_include_directories(rtklib_pinned SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB/src")
target_compile_definitions(rtklib_pinned PUBLIC
    GNSS_SIM_RTKLIB_COMMIT="${GNSS_SIM_RTKLIB_COMMIT}")
if(WIN32)
    target_link_libraries(rtklib_pinned PUBLIC winmm)
else()
    target_link_libraries(rtklib_pinned PUBLIC Threads::Threads m)
endif()

add_library(cjson_vendor STATIC
    third_party/cJSON/cJSON.c
)
add_library(gnss_sim::cjson ALIAS cjson_vendor)
target_include_directories(cjson_vendor SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/cJSON")
set_target_properties(cjson_vendor PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

add_library(gnss_sim_core STATIC
    src/core/deterministic_rng.cpp
    src/core/sim_config.cpp
    src/core/sim_time.cpp
    src/core/simulator.cpp
    src/gnss/nav_message_scheduler.cpp
    src/gnss/navigation_state.cpp
    src/gnss/rtklib_adapter.cpp
    src/gnss/satellite_engine.cpp
    src/gnss/signal_definitions.cpp
    src/model/cn0_model.cpp
    src/model/receiver_truth.cpp
    src/model/signal_tracking.cpp
)
add_library(gnss_sim::core ALIAS gnss_sim_core)
target_include_directories(gnss_sim_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(gnss_sim_core
    PRIVATE gnss_sim::rtklib gnss_sim::cjson
)
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
        tests/unit/test_cn0_model.cpp
        tests/unit/test_deterministic_rng.cpp
        tests/unit/test_nav_message_scheduler.cpp
        tests/unit/test_nav_message_scheduler_gal_bds.cpp
        tests/unit/test_navigation_state.cpp
        tests/unit/test_receiver_truth.cpp
        tests/unit/test_rtklib_adapter.cpp
        tests/unit/test_satellite_engine.cpp
        tests/unit/test_signal_definitions.cpp
        tests/unit/test_signal_tracking.cpp
        tests/unit/test_sim_config.cpp
        tests/unit/test_sim_time.cpp
    )
    target_include_directories(gnss_sim_unit_tests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_definitions(gnss_sim_unit_tests PRIVATE
        GNSS_SIM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/minimal")
    target_link_libraries(gnss_sim_unit_tests PRIVATE
        gnss_sim::core
        gnss_sim::rtklib
        GTest::gtest_main
    )
    gnss_sim_set_project_warnings(gnss_sim_unit_tests)

    include(GoogleTest)
    gtest_discover_tests(gnss_sim_unit_tests)
endif()
