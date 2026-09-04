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

set(GNSS_SIM_RTKLIB_COMMIT "c03a768a180caea7c85586fe0f3c6842f832df27" CACHE STRING
    "Pinned RTKLIB commit used by this simulator build")
set(GNSS_SIM_CJSON_COMMIT "acc76239bee01d8e9c858ae2cab296704e52d916" CACHE STRING
    "Pinned cJSON commit used by this simulator build")

execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE GNSS_SIM_GIT_RESULT
    OUTPUT_VARIABLE GNSS_SIM_SOURCE_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GNSS_SIM_GIT_RESULT EQUAL 0 OR GNSS_SIM_SOURCE_COMMIT STREQUAL "")
    set(GNSS_SIM_SOURCE_COMMIT "unknown")
endif()

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
    third_party/RTKLIB/src/ionex.c
    third_party/RTKLIB/src/pntpos.c
    third_party/RTKLIB/src/preceph.c
    third_party/RTKLIB/src/qzslex.c
    src/gnss/rtklib_rinex_obs_ext.c
    third_party/RTKLIB/src/rtcm.c
    third_party/RTKLIB/src/rtcm2.c
    third_party/RTKLIB/src/rtcm3.c
    third_party/RTKLIB/src/rtcm3e.c
    third_party/RTKLIB/src/rtkcmn.c
    third_party/RTKLIB/src/rtklib_obs_ext.c
    third_party/RTKLIB/src/rtklib_pntvel_ext.c
    third_party/RTKLIB/src/rtklib_residual_ext.c
    third_party/RTKLIB/src/rtklib_signal_state_ext.c
    third_party/RTKLIB/src/sbas.c
)
add_library(gnss_sim::rtklib ALIAS rtklib_pinned)
target_include_directories(rtklib_pinned SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB/src")
target_compile_definitions(rtklib_pinned PUBLIC
    GNSS_SIM_RTKLIB_COMMIT="${GNSS_SIM_RTKLIB_COMMIT}"
    NEXOBS=16
)
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
    src/core/atmosphere_types.cpp
    src/core/deterministic_rng.cpp
    src/core/sim_config.cpp
    src/core/sim_time.cpp
    src/core/simulator.cpp
    src/gnss/galileo_has_adapter.cpp
    src/gnss/nav_message_scheduler.cpp
    src/gnss/nav_output_metadata.cpp
    src/gnss/navigation_state.cpp
    src/gnss/rtklib_adapter.cpp
    src/gnss/rtklib_atmosphere_adapter.cpp
    src/gnss/rtklib_bias_adapter.cpp
    src/gnss/rtklib_nav_input_adapter.cpp
    src/gnss/rtklib_nav_output_adapter.cpp
    src/gnss/rtklib_output_adapter.cpp
    src/gnss/rtklib_raw_position_adapter.cpp
    src/gnss/rtklib_solution_adapter.cpp
    src/gnss/satellite_engine.cpp
    src/gnss/signal_definitions.cpp
    src/model/atmosphere_model.cpp
    src/model/bestpos_rtk_model.cpp
    src/model/carrier_tracking.cpp
    src/model/carrier_tracking_runtime.cpp
    src/model/cn0_model.cpp
    src/model/code_correlation.cpp
    src/model/code_tracking_dll.cpp
    src/model/measurement_error_model.cpp
    src/model/measurement_model.cpp
    src/model/receiver_truth.cpp
    src/model/signal_tracking.cpp
    src/model/urban_carrier_temporal.cpp
    src/model/urban_measurement_application.cpp
    src/model/urban_received_power.cpp
    src/model/urban_reflection_paths.cpp
    src/model/urban_rooftop_diffraction.cpp
    src/model/urban_rf_model.cpp
    src/model/urban_scene_geometry.cpp
    src/model/urban_signal_epoch.cpp
    src/model/urban_signal_tracking.cpp
    src/output/novatel_nav_writer.cpp
    src/output/novatel_range_writer.cpp
    src/output/novatel_solution_writer.cpp
    src/output/carrier_tracking_truth_writer.cpp
    src/output/truth_writer.cpp
    src/output/urban_truth_writer.cpp
    src/output/unicore_nav_writer.cpp
    src/scenario/scenario_engine.cpp
    src/solution/solution_engine.cpp
)
add_library(gnss_sim::core ALIAS gnss_sim_core)
target_include_directories(gnss_sim_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_definitions(gnss_sim_core PRIVATE
    GNSS_SIM_SOURCE_COMMIT="${GNSS_SIM_SOURCE_COMMIT}")
target_link_libraries(gnss_sim_core
    PRIVATE gnss_sim::rtklib gnss_sim::cjson
)
gnss_sim_set_project_warnings(gnss_sim_core)

add_executable(gnss-data-simulator
    src/app/main.cpp
)
target_link_libraries(gnss-data-simulator PRIVATE gnss_sim::core)
gnss_sim_set_project_warnings(gnss-data-simulator)

add_library(cn0_model_builder_core STATIC
    tools/build_cn0_model/cn0_builder.cpp
    tools/build_cn0_model/cn0_normalized_builder.cpp
    tools/build_cn0_model/cn0_statistics.cpp
    tools/build_cn0_model/rinex_obs_stream.cpp
)
target_include_directories(cn0_model_builder_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(cn0_model_builder_core
    PUBLIC gnss_sim::core
    PRIVATE gnss_sim::rtklib
)
gnss_sim_set_project_warnings(cn0_model_builder_core)

add_executable(build-cn0-model
    tools/build_cn0_model/main.cpp
)
target_link_libraries(build-cn0-model PRIVATE cn0_model_builder_core)
gnss_sim_set_project_warnings(build-cn0-model)

add_library(residual_validator_core STATIC
    tools/residual_validator/residual_validator.cpp
)
target_include_directories(residual_validator_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/tools/residual_validator"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(residual_validator_core
    PUBLIC gnss_sim::core
    PRIVATE gnss_sim::rtklib
)
gnss_sim_set_project_warnings(residual_validator_core)

add_executable(validate-residuals
    tools/residual_validator/main.cpp
)
target_link_libraries(validate-residuals PRIVATE residual_validator_core)
gnss_sim_set_project_warnings(validate-residuals)

add_library(rangea_roundtrip_core STATIC
    tools/rangea_roundtrip/rangea_roundtrip.cpp
    tools/rangea_roundtrip/serialized_nav_parser.cpp
)
target_include_directories(rangea_roundtrip_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/tools/rangea_roundtrip"
           "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(rangea_roundtrip_core
    PUBLIC gnss_sim::core
    PRIVATE gnss_sim::rtklib
)
gnss_sim_set_project_warnings(rangea_roundtrip_core)

add_executable(validate-rangea-roundtrip
    tools/rangea_roundtrip/main.cpp
)
target_link_libraries(validate-rangea-roundtrip PRIVATE rangea_roundtrip_core)
gnss_sim_set_project_warnings(validate-rangea-roundtrip)

add_library(transient_validator_core STATIC
    tools/transient_validator/transient_validator.cpp
)
target_include_directories(transient_validator_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/tools/transient_validator"
)
target_link_libraries(transient_validator_core
    PUBLIC rangea_roundtrip_core
)
gnss_sim_set_project_warnings(transient_validator_core)

add_executable(validate-transient-observations
    tools/transient_validator/main.cpp
)
target_link_libraries(validate-transient-observations PRIVATE transient_validator_core)
gnss_sim_set_project_warnings(validate-transient-observations)

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
        tests/unit/test_atmosphere_model.cpp
        tests/unit/test_bestpos_rtk_model.cpp
        tests/unit/test_carrier_tracking.cpp
        tests/unit/test_carrier_tracking_config.cpp
        tests/unit/test_cn0_model.cpp
        tests/unit/test_cn0_normalized_builder.cpp
        tests/unit/test_cn0_statistics.cpp
        tests/unit/test_code_correlation.cpp
        tests/unit/test_code_tracking_dll.cpp
        tests/unit/test_code_tracking_dll_boundaries.cpp
        tests/unit/test_deterministic_rng.cpp
        tests/unit/test_galileo_has_adapter.cpp
        tests/unit/test_low_elevation_signal_geometry.cpp
        tests/unit/test_measurement_error_model.cpp
        tests/unit/test_measurement_model.cpp
        tests/unit/test_nav_message_scheduler.cpp
        tests/unit/test_nav_message_scheduler_gal_bds.cpp
        tests/unit/test_nav_output_writers.cpp
        tests/unit/test_navigation_state.cpp
        tests/unit/test_output_writers.cpp
        tests/unit/test_receiver_truth.cpp
        tests/unit/test_rinex4_brd400.cpp
        tests/unit/test_rinex_obs_stream.cpp
        tests/unit/test_rtklib_adapter.cpp
        tests/unit/test_satellite_engine.cpp
        tests/unit/test_scenario_engine.cpp
        tests/unit/test_signal_correlation_metadata.cpp
        tests/unit/test_signal_definitions.cpp
        tests/unit/test_signal_tracking.cpp
        tests/unit/test_sim_config.cpp
        tests/unit/test_sim_time.cpp
        tests/unit/test_solution_engine.cpp
        tests/unit/test_solution_signal_policy.cpp
        tests/unit/test_urban_carrier_temporal.cpp
        tests/unit/test_urban_measurement_application.cpp
        tests/unit/test_urban_received_power.cpp
        tests/unit/test_urban_reflection_paths.cpp
        tests/unit/test_urban_rooftop_diffraction.cpp
        tests/unit/test_urban_rf_model.cpp
        tests/unit/test_urban_scene_geometry.cpp
        tests/unit/test_urban_signal_epoch.cpp
        tests/unit/test_urban_signal_tracking.cpp
    )
    target_include_directories(gnss_sim_unit_tests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_definitions(gnss_sim_unit_tests PRIVATE
        GNSS_SIM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/minimal")
    target_link_libraries(gnss_sim_unit_tests PRIVATE
        gnss_sim::core
        gnss_sim::rtklib
        cn0_model_builder_core
        GTest::gtest_main
    )
    gnss_sim_set_project_warnings(gnss_sim_unit_tests)

    add_executable(gnss_sim_integration_tests
        tests/integration/test_scenarios.cpp
        tests/integration/test_all_signal_residuals.cpp
        tests/integration/test_carrier_tracking_runtime.cpp
        tests/integration/test_carrier_tracking_truth.cpp
        tests/integration/test_carrier_tracking_authentic_validation.cpp
        tests/integration/test_measurement_noise_integration.cpp
        tests/integration/test_nav_equivalence.cpp
        tests/integration/test_rea_fade.cpp
        tests/integration/test_rangea_roundtrip.cpp
        tests/integration/test_transient_validator.cpp
        tests/integration/test_residual_validator.cpp
        tests/integration/test_truth_outputs.cpp
        tests/integration/test_urban_e2e_validation.cpp
        tests/integration/test_static_urban_doppler_validation.cpp
        tests/integration/test_v1_acceptance.cpp
    )
    target_include_directories(gnss_sim_integration_tests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_definitions(gnss_sim_integration_tests PRIVATE
        GNSS_SIM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/minimal")
    target_link_libraries(gnss_sim_integration_tests PRIVATE
        gnss_sim::core
        gnss_sim::rtklib
        residual_validator_core
        rangea_roundtrip_core
        transient_validator_core
        GTest::gtest_main
    )
    gnss_sim_set_project_warnings(gnss_sim_integration_tests)

    include(GoogleTest)
    gtest_discover_tests(gnss_sim_unit_tests)
    gtest_discover_tests(gnss_sim_integration_tests)

    add_test(
        NAME measurement_model_no_prng
        COMMAND ${CMAKE_COMMAND}
            -DMEASUREMENT_MODEL_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/model/measurement_model.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/check_measurement_no_rng.cmake
    )
endif()