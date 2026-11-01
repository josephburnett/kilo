#!/usr/bin/env python3
"""Flatten protocol/vectors/*.json and the generated Python module's names
into lines the C++ unit tests read without a JSON parser (stdout):

  crc <ascii-hex> <crc>
  frame <name> <MSG> <type> <seq> <payload-hex|-> <wire-hex>
  error <name> <wire-hex> <kind> [<type> <seq>]    header given when trusted
  stream <name> <bytes-hex> <expect>...           expect: ok:<type>:<seq> or err:<kind>
  enum <which> <value> <NAME>                     event, ack_status, mode, reset_cause, config_state
  reply <query-type> <reply-type>                 kilo_protocol.QUERY_REPLY
  hframe <name> <MSG> <type> <seq> <payload-hex|-> <wire-hex>   host_frames.json (the laptop channel)
  herror <name> <wire-hex> <kind>
  hcheck <name> <wire-hex> <status>               a CRC-good host frame: bad_len or unknown_type

Usage: flatten_vectors.py <repo root>
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


def main() -> int:
    repo = Path(sys.argv[1])
    vec = repo / "protocol" / "vectors"
    frames = json.loads((vec / "frames.json").read_text())
    out = []
    crc = frames["crc_check"]
    out.append(f"crc {crc['input_ascii'].encode().hex()} {crc['crc']}")
    for v in frames["frames"]:
        out.append(f"frame {v['name']} {v['msg']} {v['type']} {v['seq']} {v['payload'] or '-'} {v['wire']}")
    for v in frames["errors"]:
        head = f" {v['type']} {v['seq']}" if "type" in v else ""
        out.append(f"error {v['name']} {v['wire']} {v['error']}{head}")
    for v in frames["streams"]:
        exp = " ".join(f"ok:{e['ok']}:{e['seq']}" if "ok" in e else f"err:{e['error']}" for e in v["expect"])
        out.append(f"stream {v['name']} {v['bytes']} {exp}".rstrip())

    host = json.loads((vec / "host_frames.json").read_text())
    for v in host["frames"]:
        out.append(f"hframe {v['name']} {v['msg']} {v['type']} {v['seq']} {v['payload'] or '-'} {v['wire']}")
    out += [f"herror {v['name']} {v['wire']} {v['error']}" for v in host["errors"]]
    out += [f"hcheck {v['name']} {v['wire']} {v['status']}" for v in host["checks"]]
    out += [f"crc32 {v['name']} {v['data'] or '-'} {v['crc']}" for v in host["crc32"]]

    spec = importlib.util.spec_from_file_location("kilo_protocol", repo / "protocol" / "gen" / "kilo_protocol.py")
    P = importlib.util.module_from_spec(spec)
    sys.modules["kilo_protocol"] = P  # dataclasses look their module up while the class is built
    spec.loader.exec_module(P)
    for which, enum in (("event", P.EventCode), ("ack_status", P.AckStatus), ("mode", P.Mode),
                        ("reset_cause", P.ResetCause), ("config_state", P.ConfigState)):
        out += [f"enum {which} {int(m)} {m.name}" for m in enum]
    out += [f"reply {q} {r}" for q, r in P.QUERY_REPLY.items()]
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
