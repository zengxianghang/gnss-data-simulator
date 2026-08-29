#include "gnss/rtklib_adapter.h"
#include "gnss_sim/sim_time.h"
#include "model/measurement_model.h"
#include "output/device_marker.h"
#include "output/novatel_ascii.h"
#include "output/novatel_range_writer.h"
#include "output/novatel_solution_writer.h"
#include "solution/solution_engine.h"

#include <gtest/gtest.h>
#include <string>

namespace {

gnss_sim::SimTime writer_time() {
    return {2300, 12345678000000LL};
}

int satellite_number(const char* satellite_id) {
    int number = 0;
    EXPECT_TRUE(gnss_sim::rtklib_satellite_id_to_number(satellite_id, &number));
    return number;
}

std::string ascii_body(const std::string& message) {
    const std::size_t semicolon = message.find(';');
    const std::size_t star = message.rfind('*');
    if (semicolon == std::string::npos || star == std::string::npos || star <= semicolon) {
        return std::string();
    }
    return message.substr(semicolon + 1, star - semicolon - 1);
}

gnss_sim::MeasurementObservation observation(const char* satellite_id, gnss_sim::SignalId signal_id, int glonass_fcn,
                                             double pseudorange_m, double adr_cycles, double doppler_hz,
                                             double cn0_dbhz, double lock_time_sec) {
    gnss_sim::MeasurementObservation result{};
    result.signal_id = signal_id;
    result.satellite_number = satellite_number(satellite_id);
    result.glonass_fcn = glonass_fcn;
    result.pseudorange_m = pseudorange_m;
    result.adr_cycles = adr_cycles;
    result.doppler_hz = doppler_hz;
    result.cn0_dbhz = cn0_dbhz;
    result.lock_time_ns = static_cast<std::int64_t>(lock_time_sec * gnss_sim::NANOSECONDS_PER_SECOND + 0.5);
    result.observation_available = true;
    result.pseudorange_valid = true;
    result.doppler_valid = true;
    result.adr_valid = true;
    return result;
}

TEST(SimulatorDeviceMarker, CanonicalMarkerIsByteStable) {
    EXPECT_EQ(gnss_sim::simulator_device_marker(), "devicename=gnss-data-simulator\r\n");
}

TEST(NovatelAscii, OfficialBestPosCrcVectorMatchesOem7Documentation) {
    const std::string payload = "BESTPOSA,COM1,0,78.0,FINESTEERING,1427,325298.000,00000000,6145,2748;"
                                "SOL_COMPUTED,SINGLE,51.11678928753,-114.03886216575,1064.3470,-16.2708,WGS84,"
                                "2.3434,1.3043,4.7300,\"\",0.000,0.000,7,7,0,0,0,06,0,03";
    EXPECT_EQ(gnss_sim::novatel_ascii::crc32(payload), 0x9c9a92bbU);
}

TEST(NovatelRangeWriter, MultiConstellationRangeAIsByteStable) {
    gnss_sim::MeasurementObservation observations[] = {
        observation("C19", gnss_sim::SignalId::kBeidouB1C, 0, 22500000.250, -118000000.125000, 25.500, 43.3, 6.750),
        observation("G03", gnss_sim::SignalId::kGpsL1Ca, 0, 20200000.125, -106151716.123456, -1234.500, 45.2, 12.345),
        observation("J02", gnss_sim::SignalId::kQzssL5Q, 0, 24000000.000, -100000000.000000, -50.000, 40.0, 5.000),
        observation("R05", gnss_sim::SignalId::kGlonassG2, -4, 21400000.500, -87654321.250000, 345.125, 39.5, 3.000),
        observation("E11", gnss_sim::SignalId::kGalileoE5B, 0, 23200000.750, -121234567.750000, 678.250, 42.0, 4.500),
    };

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_rangea(writer_time(), observations, 5, &message, &error_message))
        << error_message;
    EXPECT_EQ(message, "#RANGEA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;5,"
                       "3,0,20200000.125,0.500,-106151716.123456,0.050,-1234.500,45.2,12.345,00001c04,"
                       "42,3,21400000.500,0.500,-87654321.250000,0.050,345.125,39.5,3.000,00211c04,"
                       "11,0,23200000.750,0.500,-121234567.750000,0.050,678.250,42.0,4.500,02231c04,"
                       "194,0,24000000.000,0.500,-100000000.000000,0.050,-50.000,40.0,5.000,01c51c04,"
                       "19,0,22500000.250,0.500,-118000000.125000,0.050,25.500,43.3,6.750,00e41c04"
                       "*6d3dc9c0\r\n");
}

TEST(NovatelRangeWriter, ReaSignalOffEmitsZeroObservationGoldenRecord) {
    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_rangea(writer_time(), nullptr, 0, &message, &error_message)) << error_message;
    EXPECT_EQ(message, "#RANGEA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;0*a633981e\r\n");
}

TEST(NovatelSolutionWriter, BestPosAMirrorsPsrPosBeforeRtkFix) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.valid = true;
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    solution.position.type = gnss_sim::ReceiverSolutionType::kSingle;
    solution.position.latitude_deg = 20.1;
    solution.position.longitude_deg = 120.1;
    solution.position.height_m = 101.0;
    solution.position.latitude_std_m = 0.25;
    solution.position.longitude_std_m = 0.30;
    solution.position.height_std_m = 0.50;
    solution.position.used_satellites = 6;
    gnss_sim::ReceiverTruth truth{};
    truth.latitude_deg = 20.0;
    truth.longitude_deg = 120.0;
    truth.height_m = 100.0;
    const gnss_sim::BestposRtkConfig config{true, 5000000000LL, 6, 0.01, 0.02};

    std::string bestpos;
    std::string psrpos;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(solution, 8, truth, false, config, &bestpos, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrposa(solution, 8, &psrpos, &error_message)) << error_message;
    EXPECT_EQ(ascii_body(bestpos), ascii_body(psrpos));
}

TEST(NovatelSolutionWriter, BestPosAFixedEpochUsesTruthAndNarrowInt) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.valid = true;
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    solution.position.type = gnss_sim::ReceiverSolutionType::kSingle;
    solution.position.latitude_deg = 20.1;
    solution.position.longitude_deg = 120.1;
    solution.position.height_m = 101.0;
    solution.position.latitude_std_m = 0.25;
    solution.position.longitude_std_m = 0.30;
    solution.position.height_std_m = 0.50;
    solution.position.used_satellites = 6;
    gnss_sim::ReceiverTruth truth{};
    truth.latitude_deg = 20.0;
    truth.longitude_deg = 120.0;
    truth.height_m = 100.0;
    const gnss_sim::BestposRtkConfig config{true, 5000000000LL, 6, 0.01, 0.02};

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(solution, 8, truth, true, config, &message, &error_message))
        << error_message;
    EXPECT_EQ(ascii_body(message), "SOL_COMPUTED,NARROW_INT,20.00000000000,120.00000000000,100.0000,0.0000,WGS84,"
                                   "0.0100,0.0100,0.0200,\"\",0.000,0.000,8,6,0,0,00,00,00,00");
}

TEST(NovatelSolutionWriter, InvalidBestPosMatchesInvalidPsrPosInsteadOfTruth) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    solution.position.type = gnss_sim::ReceiverSolutionType::kNone;
    gnss_sim::ReceiverTruth truth{};
    truth.latitude_deg = 20.0;
    truth.longitude_deg = 120.0;
    truth.height_m = 100.0;
    const gnss_sim::BestposRtkConfig config{true, 5000000000LL, 6, 0.01, 0.02};

    std::string bestpos;
    std::string psrpos;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_bestposa(solution, 3, truth, false, config, &bestpos, &error_message))
        << error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrposa(solution, 3, &psrpos, &error_message)) << error_message;
    EXPECT_EQ(ascii_body(bestpos), ascii_body(psrpos));
}

TEST(NovatelSolutionWriter, ValidPsrPosAIsByteStable) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.valid = true;
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    solution.position.type = gnss_sim::ReceiverSolutionType::kSingle;
    solution.position.latitude_deg = 20.0;
    solution.position.longitude_deg = 120.0;
    solution.position.height_m = 100.0;
    solution.position.latitude_std_m = 0.25;
    solution.position.longitude_std_m = 0.30;
    solution.position.height_std_m = 0.50;
    solution.position.used_satellites = 6;

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrposa(solution, 8, &message, &error_message)) << error_message;
    EXPECT_EQ(message, "#PSRPOSA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;"
                       "SOL_COMPUTED,SINGLE,20.00000000000,120.00000000000,100.0000,0.0000,WGS84,"
                       "0.2500,0.3000,0.5000,\"\",0.000,0.000,8,6,0,0,00,00,00,00*491dcb86\r\n");
}

TEST(NovatelSolutionWriter, InvalidPsrPosAIsStillScheduledAndByteStable) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    solution.position.type = gnss_sim::ReceiverSolutionType::kNone;

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrposa(solution, 3, &message, &error_message)) << error_message;
    EXPECT_EQ(message, "#PSRPOSA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;"
                       "INSUFFICIENT_OBS,NONE,0.00000000000,0.00000000000,0.0000,0.0000,WGS84,"
                       "0.0000,0.0000,0.0000,\"\",0.000,0.000,3,0,0,0,00,00,00,00*ac9735bf\r\n");
}

TEST(NovatelSolutionWriter, VelocityValidityIsIndependentOfCurrentPosition) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.position.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    solution.position.type = gnss_sim::ReceiverSolutionType::kNone;
    solution.velocity.valid = true;
    solution.velocity.status = gnss_sim::ReceiverSolutionStatus::kSolComputed;
    solution.velocity.type = gnss_sim::ReceiverSolutionType::kSingle;
    solution.velocity.horizontal_speed_mps = 12.3456;
    solution.velocity.track_over_ground_deg = 89.123456;
    solution.velocity.vertical_speed_mps = -0.25;
    solution.velocity.used_satellites = 7;

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrvela(solution, &message, &error_message)) << error_message;
    EXPECT_EQ(message, "#PSRVELA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;"
                       "SOL_COMPUTED,SINGLE,0.000,0.000,12.3456,89.123456,-0.2500,0*e2198853\r\n");
}

TEST(NovatelSolutionWriter, InvalidPsrVelAIsStillScheduledAndByteStable) {
    gnss_sim::SolutionEpoch solution{};
    solution.time = writer_time();
    solution.velocity.status = gnss_sim::ReceiverSolutionStatus::kInsufficientObs;
    solution.velocity.type = gnss_sim::ReceiverSolutionType::kNone;

    std::string message;
    std::string error_message;
    ASSERT_TRUE(gnss_sim::format_novatel_psrvela(solution, &message, &error_message)) << error_message;
    EXPECT_EQ(message, "#PSRVELA,COM1,0,0.0,FINE,2300,12345.678,00000000,0,0;"
                       "INSUFFICIENT_OBS,NONE,0.000,0.000,0.0000,0.000000,0.0000,0*e0538e06\r\n");
}

TEST(NovatelAscii, MillisecondRoundingCarriesAcrossGpsWeek) {
    const gnss_sim::SimTime time{2300, gnss_sim::GPS_WEEK_NANOSECONDS - 400000LL};
    std::string message;
    ASSERT_TRUE(gnss_sim::novatel_ascii::frame("RANGEA", time, "0", &message));
    EXPECT_NE(message.find("FINE,2301,0.000,"), std::string::npos);
}

} // namespace
