#include <iostream>

#include "gnss_sim/simulator.h"

int main()
{
    std::cout << "gnss-data-simulator " << gnss_sim::simulator_version() << '\n';
    std::cout << "RTKLIB commit: " << gnss_sim::rtklib_commit_sha() << '\n';
    return 0;
}
