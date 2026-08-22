#!/usr/bin/env python3
"""
Transport latency/packet-loss benchmark (Stage/Step 10 of the
multi-transport task). Measures round-trip PING->PONG latency over
whichever transport you point it at.

This tool is REAL and RUNNABLE. It has not been executed against real
ESP32 hardware in the environment this was written in (no boards were
attached). Do not report numbers from this tool without actually running
it — see docs/transport-options.md for the "NOT TESTED" policy.

Usage:
    python -m transports.benchmark --transport usb --port COM8
    python -m transports.benchmark --transport wifi
    python -m transports.benchmark --transport ble --count 50
"""

import argparse
import statistics
import sys
import time

from . import get_transport


def benchmark(transport_name, count=20, interval_s=0.2, **transport_kwargs):
    transport = get_transport(transport_name, **transport_kwargs)
    connected = transport.connect()

    result = {
        "transport": transport_name,
        "connected": connected,
        "sent": 0,
        "received": 0,
        "latencies_ms": [],
    }

    if not connected:
        result["note"] = "connect() failed -- NOT TESTED (no live device to measure)"
        return result

    for _ in range(count):
        transport.ping()
        result["sent"] += 1
        time.sleep(interval_s)
        latency = transport.get_latency_ms()
        if latency is not None:
            result["received"] += 1
            result["latencies_ms"].append(latency)

    transport.disconnect()
    return result


def print_report(result):
    print(f"Transport: {result['transport']}")
    print(f"Connected: {result['connected']}")
    if not result["connected"]:
        print(f"Result: NOT TESTED -- {result.get('note', 'no connection')}")
        return
    sent, received = result["sent"], result["received"]
    loss_pct = 100.0 * (sent - received) / sent if sent else 0.0
    print(f"Pings sent: {sent}, replies matched to a latency sample: {received}")
    print(f"Packet loss: {loss_pct:.1f}%")
    if result["latencies_ms"]:
        lat = result["latencies_ms"]
        print(f"Latency (ms) -- min: {min(lat):.1f}  avg: {statistics.mean(lat):.1f}  max: {max(lat):.1f}")
    else:
        print("Latency (ms): NOT TESTED -- no replies received")


def main():
    parser = argparse.ArgumentParser(description="Haptic transport latency/packet-loss benchmark")
    parser.add_argument("--transport", required=True, choices=["usb", "wifi", "ble", "bluetooth", "espnow"])
    parser.add_argument("--port", help="USB: COM port")
    parser.add_argument("--host", help="Wi-Fi: ESP32 IP address")
    parser.add_argument("--count", type=int, default=20, help="Number of pings to send")
    parser.add_argument("--interval", type=float, default=0.2, help="Seconds between pings")
    args = parser.parse_args()

    kwargs = {}
    if args.port:
        kwargs["port"] = args.port
    if args.host:
        kwargs["host"] = args.host

    result = benchmark(args.transport, count=args.count, interval_s=args.interval, **kwargs)
    print_report(result)


if __name__ == "__main__":
    main()
