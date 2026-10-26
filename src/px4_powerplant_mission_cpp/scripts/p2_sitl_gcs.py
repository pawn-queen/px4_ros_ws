#!/usr/bin/env python3
"""Loopback-only GCS link for the isolated P2 simulator, never a flight agent.

The client first receives a real PX4 autopilot heartbeat on the configured SITL
GCS port, then replies to that UDP peer with GCS heartbeats. It sends no arming,
mode, actuator or trajectory commands. PX4's normal data-link checks remain on.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import socket
import struct
import time

from pymavlink.dialects.v20 import common as mavlink

READ_PARAMETERS = ("NAV_DLL_ACT", "COM_DL_LOSS_T", "COM_SPOOLUP_TIME", "MPC_TKO_RAMP_T")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--listen-port", type=int, default=14550)
    parser.add_argument("--expected-px4-port", type=int, default=18570)
    parser.add_argument("--startup-timeout", type=float, default=120)
    args = parser.parse_args()
    if (not 1024 <= args.listen_port <= 65535 or not 1024 <= args.expected_px4_port <= 65535
            or args.listen_port == args.expected_px4_port or args.startup_timeout <= 0):
        parser.error("distinct unprivileged ports and a positive timeout are required")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    health_path = args.output_dir / "gcs_health.json"
    events = (args.output_dir / "gcs_heartbeats.jsonl").open("w", buffering=1)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    endpoint = ("127.0.0.1", args.expected_px4_port)
    connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # No SO_REUSEADDR: an existing GCS listener is not shared or displaced.
    connection.bind(("127.0.0.1", args.listen_port))
    connection.settimeout(.05)
    peer = None
    vehicle_system = vehicle_component = None

    class Writer:
        def write(self, packet):
            if peer is None:
                raise RuntimeError("no verified PX4 peer")
            connection.sendto(packet, peer)

    protocol = mavlink.MAVLink(Writer(), srcSystem=255, srcComponent=190)
    protocol.robust_parsing = True
    started = time.monotonic()
    received = sent = rejected_peers = invalid_packets = 0
    last_autopilot = last_sent = first_autopilot = None
    last_report = 0
    parameters = {}
    failure = None

    def snapshot(now):
        age = now-last_autopilot if last_autopilot is not None else None
        return {"running": running, "updated_wall_monotonic": now,
                "transport": "MAVLink UDP loopback", "listen_endpoint": ["127.0.0.1", args.listen_port],
                "expected_px4_endpoint": list(endpoint), "verified_px4_endpoint": list(peer) if peer else None,
                "verified_vehicle_system_id": vehicle_system, "verified_vehicle_component_id": vehicle_component,
                "first_autopilot_heartbeat_wall": first_autopilot,
                "last_autopilot_heartbeat_wall": last_autopilot, "autopilot_heartbeat_age_wall_s": age,
                "autopilot_heartbeats_received": received, "gcs_heartbeats_sent": sent,
                "last_gcs_heartbeat_wall": last_sent, "gcs_heartbeat_period_wall_s": 1.0,
                "link_healthy": bool(running and received >= 2 and sent >= 2 and age is not None and age <= 3),
                "read_only_px4_parameters": parameters, "rejected_udp_peers": rejected_peers,
                "invalid_mavlink_packets": invalid_packets, "failure": failure,
                "emits_arming_mode_or_trajectory_commands": False,
                "preserves_px4_datalink_checks": True}

    def save(now):
        row = snapshot(now)
        temporary = health_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(row, indent=2, allow_nan=False))
        temporary.replace(health_path)
        events.write(json.dumps(row, allow_nan=False)+"\n")

    try:
        while running:
            try:
                packet, address = connection.recvfrom(65535)
            except socket.timeout:
                packet, address = None, None
            now = time.monotonic()
            if packet is not None:
                if address != endpoint:
                    rejected_peers += 1
                else:
                    try:
                        messages = protocol.parse_buffer(packet) or []
                    except mavlink.MAVError:
                        invalid_packets += 1
                        messages = []
                    for message in messages:
                        if message.get_type() == "BAD_DATA":
                            invalid_packets += 1
                        elif (message.get_type() == "HEARTBEAT"
                              and message.autopilot == mavlink.MAV_AUTOPILOT_PX4
                              and message.type != mavlink.MAV_TYPE_GCS
                              and message.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1):
                            if peer is None:
                                peer = address
                                vehicle_system, vehicle_component = message.get_srcSystem(), message.get_srcComponent()
                                first_autopilot = now
                            if (message.get_srcSystem(), message.get_srcComponent()) == (vehicle_system, vehicle_component):
                                received += 1
                                last_autopilot = now
                        elif (peer is not None and message.get_type() == "PARAM_VALUE"
                              and (message.get_srcSystem(), message.get_srcComponent()) == (vehicle_system, vehicle_component)):
                            name = message.param_id
                            if isinstance(name, bytes):
                                name = name.decode("ascii", errors="replace")
                            name = name.rstrip("\0")
                            if name in READ_PARAMETERS:
                                raw = float(message.param_value)
                                # This checkout uses the MAVLink bytewise PX4
                                # parameter encoding for integer parameters.
                                value = (struct.unpack("<i", struct.pack("<f", raw))[0]
                                         if message.param_type == mavlink.MAV_PARAM_TYPE_INT32 else raw)
                                parameters[name] = {"value": value, "encoded_float": raw, "type": message.param_type,
                                                    "received_wall_monotonic": now}
            if peer is not None and (last_sent is None or now-last_sent >= 1):
                protocol.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID,
                                        0, 0, mavlink.MAV_STATE_ACTIVE)
                sent += 1
                last_sent = now
                # Read-only requests preserve the actual PX4 data-link policy in
                # the evidence. No parameter is changed to bypass preflight.
                for name in READ_PARAMETERS:
                    if name not in parameters:
                        protocol.param_request_read_send(vehicle_system, vehicle_component,
                                                         name.encode("ascii"), -1)
            if now-last_report >= 1:
                save(now)
                last_report = now
            if peer is None and now-started > args.startup_timeout:
                failure = "no verified PX4 autopilot heartbeat before startup timeout"
                break
            if last_autopilot is not None and now-last_autopilot > 5:
                failure = "PX4 autopilot heartbeat stale for more than five wall seconds"
                break
    except BaseException as error:
        failure = type(error).__name__+": "+str(error)
    finally:
        running = False
        save(time.monotonic())
        connection.close()
        events.close()
    if failure:
        print(failure, flush=True)
    return int(failure is not None)


if __name__ == "__main__":
    raise SystemExit(main())
