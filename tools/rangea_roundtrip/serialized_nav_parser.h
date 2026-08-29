#ifndef GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_SERIALIZED_NAV_PARSER_H_
#define GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_SERIALIZED_NAV_PARSER_H_

#include "gnss/nav_output_record.h"

#include <string>

namespace gnss_sim {

struct ParsedSerializedNavRecord {
    int output_gps_week;
    double output_sow_sec;
    NavOutputRecord record;
};

bool parse_serialized_novatel_nav_line_independent(const std::string& raw_line, ParsedSerializedNavRecord* parsed,
                                                   bool* recognized, std::string* error_message);

} // namespace gnss_sim

#endif // GNSS_SIM_TOOLS_RANGEA_ROUNDTRIP_SERIALIZED_NAV_PARSER_H_
