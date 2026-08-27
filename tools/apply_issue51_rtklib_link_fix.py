from pathlib import Path

path = Path("CMakeLists.txt")
text = path.read_text()
old = "    third_party/RTKLIB/src/rtklib_residual_ext.c\n    third_party/RTKLIB/src/sbas.c\n"
new = "    third_party/RTKLIB/src/rtklib_residual_ext.c\n    third_party/RTKLIB/src/rtklib_signal_state_ext.c\n    third_party/RTKLIB/src/sbas.c\n"
count = text.count(old)
if count != 1:
    raise RuntimeError(f"CMakeLists.txt: expected one residual-source match, found {count}")
path.write_text(text.replace(old, new, 1))
print("RTKLIB signal-state link source added")
