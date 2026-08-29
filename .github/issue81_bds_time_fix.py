from pathlib import Path

adapter = Path('src/gnss/rtklib_nav_input_adapter.cpp')
text = adapter.read_text()
old = '    eph.toes = source.toe_sow_sec;\n'
new = '''    // Serialized BD2EPHEMA Toe is GPST by the receiver log contract, but
    // RTKLIB keeps eph.toes in the native BDT seconds-of-week for BeiDou.
    // eph.toe remains the absolute GPST epoch; only the raw orbital Toe SOW
    // used by eph2pos()'s Earth-rotation term needs the GPST-BDT 14 s offset.
    eph.toes = source.toe_sow_sec;
    if (system == SYS_CMP) {
        eph.toes -= 14.0;
        if (eph.toes < 0.0) {
            eph.toes += 604800.0;
        }
    }
'''
assert old in text
adapter.write_text(text.replace(old, new, 1))

writer = Path('src/output/novatel_nav_writer.cpp')
text = writer.read_text()
old = '''                if (eph.message_family != RtklibBroadcastMessageFamily::kGalileoInav &&
                    !(eph.message_family == RtklibBroadcastMessageFamily::kGalileoFnav && !eph.galileo_inav_received)) {
'''
new = '''                if (eph.message_family != RtklibBroadcastMessageFamily::kGalileoInav &&
                    !(eph.message_family == RtklibBroadcastMessageFamily::kGalileoFnav &&
                      !eph.galileo_inav_received)) {
'''
assert old in text
writer.write_text(text.replace(old, new, 1))
