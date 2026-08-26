file(READ "${MEASUREMENT_MODEL_SOURCE}" measurement_source)

string(FIND "${measurement_source}" "DeterministicRng" deterministic_rng_pos)
string(FIND "${measurement_source}" "rng_next_" rng_call_pos)
string(FIND "${measurement_source}" "std::random" std_random_pos)

if(NOT deterministic_rng_pos EQUAL -1 OR NOT rng_call_pos EQUAL -1 OR NOT std_random_pos EQUAL -1)
    message(FATAL_ERROR "V1 measurement model must not call a PRNG")
endif()
