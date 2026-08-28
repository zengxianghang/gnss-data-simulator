from pathlib import Path


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one guarded match, found {count}")
    p.write_text(text.replace(old, new, 1))


replace_once(
    "src/model/measurement_model.cpp",
    '''    const bool geometry_usable = geometry.visible;
    const bool code_bias_available = result.code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    result.observation_available = geometry_usable && tracker.observation_available;
    result.pseudorange_valid = geometry_usable && tracker.psr_valid && code_bias_available;
    result.doppler_valid = geometry_usable && tracker.doppler_valid;
    result.adr_valid = geometry_usable && tracker.adr_valid && ambiguity_state->initialized;''',
    '''    // RANGE measurement validity describes RF tracking/measurement quality,
    // not whether the broadcast ephemeris is healthy for navigation.  Keep
    // broadcast health in geometry.healthy/geometry.visible for PVT gating,
    // while an above-mask tracked signal may still produce raw PSR/Doppler/ADR.
    const bool measurement_geometry_usable = geometry.above_elevation_mask;
    const bool code_bias_available = result.code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    result.observation_available = measurement_geometry_usable && tracker.observation_available;
    result.pseudorange_valid = measurement_geometry_usable && tracker.psr_valid && code_bias_available;
    result.doppler_valid = measurement_geometry_usable && tracker.doppler_valid;
    result.adr_valid = measurement_geometry_usable && tracker.adr_valid && ambiguity_state->initialized;''',
)

replace_once(
    "tests/unit/test_measurement_model.cpp",
    '''TEST(ZeroNoiseMeasurement, MultiFrequencySharesGeometryClockAndDiffersByWavelengthAndBias) {''',
    '''TEST(ZeroNoiseMeasurement, BroadcastHealthDoesNotInvalidateTrackedRawMeasurement) {
    NavGuard nav{gnss_sim::create_rtklib_nav_store()};
    ASSERT_NE(nav.store, nullptr);
    std::string error_message;
    ASSERT_TRUE(gnss_sim::load_rinex_nav_file(nav.store, mixed_nav_path().c_str(), &error_message)) << error_message;

    gnss_sim::ReceiverTruth receiver{};
    ASSERT_TRUE(make_test_receiver(&receiver, &error_message));
    gnss_sim::SimTime receive_time{};
    ASSERT_TRUE(gnss_sim::sim_time_from_week_sow(2041, 180100.0, &receive_time));
    int satellite_number = 0;
    ASSERT_TRUE(gnss_sim::rtklib_satellite_id_to_number("G01", &satellite_number));
    gnss_sim::SatelliteGeometry geometry{};
    ASSERT_TRUE(gnss_sim::compute_satellite_geometry(nav.store, receiver, receive_time, satellite_number, -90.0,
                                                     &geometry, &error_message))
        << error_message;
    ASSERT_TRUE(geometry.above_elevation_mask);
    geometry.healthy = false;
    geometry.visible = false;

    gnss_sim::SimTime tracking_start{};
    ASSERT_TRUE(gnss_sim::add_time_ns(receive_time, -10 * gnss_sim::NANOSECONDS_PER_SECOND, &tracking_start));
    gnss_sim::SignalTracker tracker = tracking_tracker(gnss_sim::SignalId::kGpsL1Ca, tracking_start);
    gnss_sim::AtmosphereCorrection atmosphere{};
    atmosphere.mode = gnss_sim::AtmosphereMode::BROADCAST;
    atmosphere.ionosphere_code_delay_m = 0.0;
    atmosphere.troposphere_delay_m = 0.0;
    gnss_sim::CarrierAmbiguityState ambiguity{};
    gnss_sim::MeasurementObservation observation{};
    ASSERT_TRUE(gnss_sim::generate_zero_noise_measurement(nav.store, geometry, receiver, tracker, atmosphere, &ambiguity,
                                                          &observation, &error_message))
        << error_message;

    EXPECT_TRUE(observation.observation_available);
    EXPECT_TRUE(observation.pseudorange_valid);
    EXPECT_TRUE(observation.doppler_valid);
    EXPECT_TRUE(observation.adr_valid);
}

TEST(ZeroNoiseMeasurement, MultiFrequencySharesGeometryClockAndDiffersByWavelengthAndBias) {''',
)

print("raw RANGE measurement validity is separated from broadcast navigation health")
