#ifndef GNSS_SIM_SRC_OUTPUT_UNICORE_NAV_WRITER_H_
#define GNSS_SIM_SRC_OUTPUT_UNICORE_NAV_WRITER_H_

#include "gnss/nav_output_record.h"
#include "gnss_sim/sim_types.h"

#include <string>

namespace gnss_sim {

bool format_unicore_nav_output_record(const NavOutputRecord& record, const SimTime& output_time, std::string* message,
                                      bool* supported, std::string* error_message);
bool format_unicore_receiver_nav_record(const RtklibNavStore* receiver_nav, int output_record_index,
                                        const SimTime& output_time, std::string* message, bool* supported,
                                        std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_SRC_OUTPUT_UNICORE_NAV_WRITER_H_
