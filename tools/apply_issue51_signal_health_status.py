from pathlib import Path

path = Path("src/gnss/rtklib_bias_adapter.cpp")
text = path.read_text()
old = '''    if (status <= 0) {
        set_error(error_message, "no matching signal/message-family ephemeris for health");
        return false;
    }

    const int raw_health = info.system == SYS_GLO ? geph.svh : eph.svh;
'''
new = '''    if (status == 0) {
        *signal_health = 1;
        return true;
    }
    if (status < 0) {
        set_error(error_message, "signal/message-family status lookup failed");
        return false;
    }

    const int raw_health = info.system == SYS_GLO ? geph.svh : eph.svh;
'''
count = text.count(old)
if count != 1:
    raise RuntimeError(f"signal status anchor count={count}")
path.write_text(text.replace(old, new, 1))
print("missing signal family is non-fatal unavailable status")
