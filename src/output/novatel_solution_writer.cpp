#include "output/novatel_solution_writer.h"

#include "output/novatel_ascii.h"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace gnss_sim {
namespace {

void set_error(std::string* error_message, const char* message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool consistent_position(const PositionSolution& position) {
    if (position.valid) {
        return position.status == ReceiverSolutionStatus::kSolComputed &&
               position.type == ReceiverSolutionType::kSingle && std::isfinite(position.latitude_deg) &&
               std::isfinite(position.longitude_deg) && std::isfinite(position.height_m) &&
               std::isfinite(position.latitude_std_m) && std::isfinite(position.longitude_std_m) &&
               std::isfinite(position.height_std_m) && position.latitude_std_m >= 0.0 &&
               position.longitude_std_m >= 0.0 && position.height_std_m >= 0.0 && position.used_satellites >= 0 &&
               position.used_satellites <= 255;
    }
    return position.status == ReceiverSolutionStatus::kInsufficientObs && position.type == ReceiverSolutionType::kNone;
}

bool consistent_velocity(const VelocitySolution& velocity) {
    if (velocity.valid) {
        return velocity.status == ReceiverSolutionStatus::kSolComputed &&
               velocity.type == ReceiverSolutionType::kSingle && std::isfinite(velocity.horizontal_speed_mps) &&
               velocity.horizontal_speed_mps >= 0.0 && std::isfinite(velocity.track_over_ground_deg) &&
               velocity.track_over_ground_deg >= 0.0 && velocity.track_over_ground_deg < 360.0 &&
               std::isfinite(velocity.vertical_speed_mps) && velocity.used_satellites >= 0 &&
               velocity.used_satellites <= 255;
    }
    return velocity.status == ReceiverSolutionStatus::kInsufficientObs && velocity.type == ReceiverSolutionType::kNone;
}

} // namespace

bool format_novatel_psrposa(const SolutionEpoch& solution, int tracked_satellites, std::string* message,
                            std::string* error_message) {
    if (message == nullptr || tracked_satellites < 0 || tracked_satellites > 255 ||
        !consistent_position(solution.position)) {
        set_error(error_message, "PSRPOSA solution metadata is invalid");
        return false;
    }

    const PositionSolution& position = solution.position;
    const double latitude_deg = position.valid ? position.latitude_deg : 0.0;
    const double longitude_deg = position.valid ? position.longitude_deg : 0.0;
    const double height_m = position.valid ? position.height_m : 0.0;
    const double latitude_std_m = position.valid ? position.latitude_std_m : 0.0;
    const double longitude_std_m = position.valid ? position.longitude_std_m : 0.0;
    const double height_std_m = position.valid ? position.height_std_m : 0.0;
    const int used_satellites = position.valid ? position.used_satellites : 0;

    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << receiver_solution_status_name(position.status) << ',' << receiver_solution_type_name(position.type) << ','
         << std::fixed << std::setprecision(11) << latitude_deg << ',' << longitude_deg << ',' << std::setprecision(4)
         << height_m << ",0.0000,WGS84," << latitude_std_m << ',' << longitude_std_m << ',' << height_std_m
         << ",\"\",0.000,0.000," << tracked_satellites << ',' << used_satellites << ",0,0,00,00,00,00";

    if (!novatel_ascii::frame("PSRPOSA", solution.time, body.str(), message)) {
        set_error(error_message, "PSRPOSA header time cannot be represented");
        return false;
    }
    return true;
}

bool format_novatel_psrvela(const SolutionEpoch& solution, std::string* message, std::string* error_message) {
    if (message == nullptr || !consistent_velocity(solution.velocity)) {
        set_error(error_message, "PSRVELA solution metadata is invalid");
        return false;
    }

    const VelocitySolution& velocity = solution.velocity;
    const double horizontal_speed_mps = velocity.valid ? velocity.horizontal_speed_mps : 0.0;
    const double track_over_ground_deg = velocity.valid ? velocity.track_over_ground_deg : 0.0;
    const double vertical_speed_mps = velocity.valid ? velocity.vertical_speed_mps : 0.0;

    std::ostringstream body;
    body.imbue(std::locale::classic());
    body << receiver_solution_status_name(velocity.status) << ',' << receiver_solution_type_name(velocity.type)
         << ",0.000,0.000," << std::fixed << std::setprecision(4) << horizontal_speed_mps << ',' << std::setprecision(6)
         << track_over_ground_deg << ',' << std::setprecision(4) << vertical_speed_mps << ",0";

    if (!novatel_ascii::frame("PSRVELA", solution.time, body.str(), message)) {
        set_error(error_message, "PSRVELA header time cannot be represented");
        return false;
    }
    return true;
}

} // namespace gnss_sim
