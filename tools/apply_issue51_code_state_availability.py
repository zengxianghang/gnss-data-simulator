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
    '''    RtklibSatelliteState code_state = geometry.satellite_state;
    double code_geometric_range_m = geometry.geometric_range_m;
    if (signal_family_bias_available) {''',
    '''    double selected_code_bias_m = 0.0;
    BroadcastCodeBiasStatus selected_code_bias_status = BroadcastCodeBiasStatus::kUnavailableForMessageFamily;
    if (signal_family_bias_available &&
        !compute_broadcast_code_bias_m(*signal, bias_data, &selected_code_bias_m, &selected_code_bias_status,
                                       error_message)) {
        return false;
    }
    const bool family_code_bias_available =
        signal_family_bias_available &&
        selected_code_bias_status != BroadcastCodeBiasStatus::kUnavailableForMessageFamily;

    RtklibSatelliteState code_state = geometry.satellite_state;
    double code_geometric_range_m = geometry.geometric_range_m;
    if (family_code_bias_available) {''',
)

replace_once(
    "src/model/measurement_model.cpp",
    '''    if (!signal_family_bias_available) {
        set_unavailable(&result.code_bias_m, &result.code_bias_status);
    } else if (!compute_broadcast_code_bias_m(*signal, bias_data, &result.code_bias_m, &result.code_bias_status,
                                              error_message)) {
        return false;
    }''',
    '''    if (!signal_family_bias_available) {
        set_unavailable(&result.code_bias_m, &result.code_bias_status);
    } else {
        result.code_bias_m = selected_code_bias_m;
        result.code_bias_status = selected_code_bias_status;
    }''',
)

print("family code state now requires an available code-bias model")
