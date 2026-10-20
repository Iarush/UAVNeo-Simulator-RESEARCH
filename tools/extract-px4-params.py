"""Extract PX4 parameters from QGroundControl .tlog files into a QGC .params file.

Usage:
    python tools/extract-px4-params.py OUT.params LOG.tlog [LOG.tlog ...]

Each tlog record is an 8-byte big-endian microsecond timestamp followed by one MAVLink
v1 (0xFE) or v2 (0xFD) packet. Only two messages are decoded, both CRC-checked:
  - PARAM_VALUE (22): float value, uint16 count, uint16 index, char[16] id, uint8 type.
    MAVLink v2 trims trailing zero bytes, so the payload is padded back to 25 bytes.
    PX4 sends INT32 (type 6) parameters as raw bits in the float field.
  - AUTOPILOT_VERSION (148): uint64 capabilities, uint64 uid, uint32 flight_sw_version, ...

Logs are applied oldest first (by first timestamp), so the newest log's value wins.
Logs from different vehicles (different AUTOPILOT_VERSION uid) are refused.
Used by the sim's flight model (docs: .plans/real-flight-physics-spec.md, section 4.1).
"""

import struct
import sys
from pathlib import Path

MSG_PARAM_VALUE = 22
MSG_AUTOPILOT_VERSION = 148
CRC_EXTRA = {MSG_PARAM_VALUE: 220, MSG_AUTOPILOT_VERSION: 178}
PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL32 = 9


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def packets(data):
    """Yield (timestamp_us, sysid, msgid, payload) for CRC-valid wanted messages."""
    i = 0
    n = len(data)
    while i + 9 < n:
        magic = data[i + 8]
        if magic == 0xFE and i + 8 + 8 <= n:
            plen = data[i + 9]
            head, body = 6, i + 8
            msgid = data[body + 5]
            sysid = data[body + 3]
        elif magic == 0xFD and i + 8 + 12 <= n:
            plen = data[i + 9]
            incompat = data[i + 10]
            head, body = 10, i + 8
            msgid = data[body + 7] | (data[body + 8] << 8) | (data[body + 9] << 16)
            sysid = data[body + 5]
        else:
            i += 1  # resync
            continue
        sig = 13 if magic == 0xFD and (incompat & 1) else 0
        end = body + head + plen + 2 + sig
        if end > n:
            break
        if msgid in CRC_EXTRA:
            crc = x25(data[body + 1:body + head + plen])
            crc = x25([CRC_EXTRA[msgid]], crc)
            got = data[body + head + plen] | (data[body + head + plen + 1] << 8)
            if crc != got:
                i += 1
                continue
            ts = struct.unpack(">Q", data[i:i + 8])[0]
            yield ts, sysid, msgid, data[body + head:body + head + plen]
        i = end


def read_log(path):
    data = Path(path).read_bytes()
    params, uid, version, first_ts = {}, None, None, None
    for ts, sysid, msgid, payload in packets(data):
        if first_ts is None:
            first_ts = ts
        if msgid == MSG_PARAM_VALUE:
            p = payload.ljust(25, b"\0")
            value_bits = p[0:4]
            name = p[8:24].split(b"\0", 1)[0].decode("ascii", "replace")
            ptype = p[24]
            if ptype == PARAM_TYPE_INT32:
                value = struct.unpack("<i", value_bits)[0]
            else:
                value = struct.unpack("<f", value_bits)[0]
            params[name] = (value, ptype, sysid)
        elif msgid == MSG_AUTOPILOT_VERSION:
            p = payload.ljust(60, b"\0")
            uid = struct.unpack("<Q", p[8:16])[0]
            version = struct.unpack("<I", p[16:20])[0]
    return {"path": path, "params": params, "uid": uid, "version": version, "first_ts": first_ts or 0}


def version_string(v):
    if v is None:
        return "unknown"
    kinds = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "official"}
    return f"v{(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF} ({kinds.get(v & 0xFF, v & 0xFF)})"


def format_value(value, ptype):
    if ptype == PARAM_TYPE_INT32 or isinstance(value, int):
        return str(int(value))
    return f"{value:.9g}"


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    out = Path(argv[1])
    logs = [read_log(p) for p in argv[2:]]
    logs = [l for l in logs if l["params"]]
    if not logs:
        print("error: no PARAM_VALUE messages found in any log", file=sys.stderr)
        return 1

    uids = {l["uid"] for l in logs if l["uid"] is not None}
    if len(uids) > 1:
        print("error: logs come from different vehicles (AUTOPILOT_VERSION uid " +
              ", ".join(f"{u:#x}" for u in sorted(uids)) + "); pass one vehicle's logs only", file=sys.stderr)
        return 1
    versions = {l["version"] for l in logs if l["version"] is not None}

    logs.sort(key=lambda l: l["first_ts"])
    merged = {}
    for l in logs:
        merged.update(l["params"])
        print(f"{l['path']}: {len(l['params'])} parameters, PX4 {version_string(l['version'])}")

    newest = logs[-1]
    sysid = next(iter(newest["params"].values()))[2]
    lines = [
        f"# Onboard parameters for Vehicle {sysid}",
        f"# PX4 {', '.join(version_string(v) for v in sorted(versions)) or 'unknown'}",
        f"# Extracted by tools/extract-px4-params.py from {len(logs)} log(s); newest: {Path(newest['path']).name}",
        "#",
        "# Vehicle-Id Component-Id Name Value Type",
    ]
    for name in sorted(merged):
        value, ptype, sid = merged[name]
        lines.append(f"{sid}\t1\t{name}\t{format_value(value, ptype)}\t{ptype}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {len(merged)} parameters to {out}")
    print(f"PX4 {', '.join(version_string(v) for v in sorted(versions))}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
