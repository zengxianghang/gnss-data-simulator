from pathlib import Path

path = Path("tests/unit/test_solution_engine.cpp")
text = path.read_text()
old = '''gnss_sim::generate_zero_noise_measurement(truth_nav, geometry, tracker, atmosphere, &ambiguity,
                                                       &measurements[index], error_message)'''
new = '''gnss_sim::generate_zero_noise_measurement(truth_nav, geometry, receiver, tracker, atmosphere, &ambiguity,
                                                       &measurements[index], error_message)'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"test_solution_engine signature anchor: expected 1, found {count}")
path.write_text(text.replace(old, new, 1))
print("solution-engine measurement signature updated")
