#!/usr/bin/env python3

import argparse
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import serial


BAUD = 115200
CONTROL_MAGIC = 0xC0
CONTROL_BROADCAST_NODE_ID = 0xFF

CMD_SET_RATE = 1
CMD_SET_RADIO_PARAMS = 4
CMD_START = 5
CMD_STOP = 6
CMD_SET_GFSK_PARAMS = 9
CMD_SET_LORA_PARAMS = 10
CMD_SET_NODE_COUNT = 11  # BLE RX-local only — never relayed to a TX board.
                          # Reuses the intervalMs field to carry the desired
                          # simultaneous-connection count (0..MAX_TX_NODES).

PORTS = {
    "espnow": "/dev/espnow_rx",
    "gfsk": "/dev/gfsk_rx",
    "lora": "/dev/lora_rx",
    "ble": "/dev/ble_rx",
}


# ControlPacket:
#   uint8_t magic
#   uint8_t cmd
#   uint8_t targetNodeId
#   uint8_t reserved0
#   uint32_t intervalMs
#   float freqMHz
#   GfskParams:
#       uint32_t bitrateBps
#       uint16_t preambleBits
#       uint8_t deviationKhz
#       uint8_t rxBwIndex
#       uint8_t reserved
#   LoraParams:
#       uint8_t sf
#       uint8_t bwIndex
#       uint8_t crIndex
#       uint8_t syncWord
#       int8_t txPowerDbm
#       uint8_t preambleSymbols
#       uint8_t reserved[2]
#   uint16_t crc
WITHOUT_CRC_FORMAT = "<4BIfI H 3B 4B b B 2B"
PACKET_FORMAT = WITHOUT_CRC_FORMAT + "H"


def make_packet(
    cmd,
    target_node_id=CONTROL_BROADCAST_NODE_ID,
    interval_ms=100,
    freq_mhz=868.0,
    gfsk_bitrate_bps=50000,
    gfsk_preamble_bits=128,
    gfsk_deviation_khz=25,
    gfsk_rx_bw_index=15,
    lora_sf=7,
    lora_bw_index=7,
    lora_cr_index=0,
    lora_sync_word=0x12,
    lora_tx_power_dbm=14,
    lora_preamble_symbols=8,
):
    """Pack one command into the fixed format used by the firmware."""
    values = (
        CONTROL_MAGIC,
        cmd,
        target_node_id,
        0,
        interval_ms,
        freq_mhz,
        gfsk_bitrate_bps,
        gfsk_preamble_bits,
        gfsk_deviation_khz,
        gfsk_rx_bw_index,
        0,
        lora_sf,
        lora_bw_index,
        lora_cr_index,
        lora_sync_word,
        lora_tx_power_dbm,
        lora_preamble_symbols,
        0,
        0,
    )

    without_crc = struct.pack(WITHOUT_CRC_FORMAT, *values)
    crc = sum(without_crc) & 0xFFFF

    return struct.pack(PACKET_FORMAT, *values, crc)


def send_packet(port, packet):
    """Send the command a few times, then print any RX replies."""
    with serial.Serial(port, BAUD, timeout=1) as ser:
        # The RX board may reset when the serial port opens.
        time.sleep(2.2)
        ser.reset_input_buffer()

        for _ in range(3):
            ser.write(packet)
            ser.flush()
            time.sleep(0.05)

        deadline = time.time() + 0.75
        while time.time() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(line)


def capture_start(port, packet, seconds, protocol, node):
    """Log RX output for a timed run, then send STOP and save the tail."""
    log_dir = Path.home() / "radio_logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    node_label = "all" if node == CONTROL_BROADCAST_NODE_ID else str(node)
    log_path = log_dir / f"{protocol}_{timestamp}_node-{node_label}.log"
    stop_packet = make_packet(CMD_STOP, target_node_id=node)

    print(f"Logging RX output to {log_path}")
    with serial.Serial(port, BAUD, timeout=0.2) as ser:
        # Opening the RX serial port may reset the board.
        time.sleep(2.2)
        ser.reset_input_buffer()

        with log_path.open("w", encoding="utf-8", buffering=1) as log:
            log.write(f"# protocol={protocol} port={port} node={node_label}\n")
            log.write(f"# duration_seconds={seconds} started={datetime.now().isoformat()}\n")

            try:
                for _ in range(3):
                    ser.write(packet)
                    ser.flush()
                    time.sleep(0.05)

                print(f"Capture running for {seconds} seconds; STOP will be sent afterward.")
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    raw = ser.readline()
                    if raw:
                        text = raw.decode(errors="replace")
                        print(text, end="", flush=True)
                        log.write(text)
            finally:
                print("Capture duration complete; sending STOP.")
                for _ in range(3):
                    ser.write(stop_packet)
                    ser.flush()
                    time.sleep(0.05)

                # Keep logging the RX's relay messages and any packets in flight.
                stop_deadline = time.monotonic() + 3
                while time.monotonic() < stop_deadline:
                    raw = ser.readline()
                    if raw:
                        text = raw.decode(errors="replace")
                        print(text, end="", flush=True)
                        log.write(text)

    print(f"Log saved: {log_path}")


def positive_seconds(value):
    """Accept only a positive capture duration."""
    seconds = int(value)
    if seconds <= 0:
        raise argparse.ArgumentTypeError("must be a positive number of seconds")
    return seconds


def build_parser():
    """Define the command-line options for each radio operation."""
    parser = argparse.ArgumentParser()

    parser.add_argument("--protocol", choices=PORTS.keys())
    parser.add_argument(
        "--port",
        default=None,
        help="Override the serial port directly",
    )

    sub = parser.add_subparsers(dest="action", required=True)

    p_rate = sub.add_parser("rate")
    p_rate.add_argument("interval_ms", type=int)
    p_rate.add_argument("--node", type=int, default=CONTROL_BROADCAST_NODE_ID)

    p_start = sub.add_parser("start")
    p_start.add_argument("--node", type=int, default=CONTROL_BROADCAST_NODE_ID)
    p_start.add_argument(
        "--log",
        type=positive_seconds,
        metavar="SECONDS",
        help="capture RX serial output for this many seconds, then send STOP",
    )

    p_stop = sub.add_parser("stop")
    p_stop.add_argument("--node", type=int, default=CONTROL_BROADCAST_NODE_ID)

    p_gfsk = sub.add_parser("gfsk")
    p_gfsk.add_argument("--node", type=int, default=CONTROL_BROADCAST_NODE_ID)
    p_gfsk.add_argument("--freq", type=float, default=868.0)
    p_gfsk.add_argument("--bitrate", type=int, default=50000)
    p_gfsk.add_argument("--deviation", type=int, default=25)
    p_gfsk.add_argument("--rxbw-index", type=int, default=15)
    p_gfsk.add_argument("--preamble-bits", type=int, default=128)

    p_lora = sub.add_parser("lora")
    p_lora.add_argument("--node", type=int, default=CONTROL_BROADCAST_NODE_ID)
    p_lora.add_argument("--freq", type=float, default=868.0)
    p_lora.add_argument("--sf", type=int, default=7)
    p_lora.add_argument("--bw-index", type=int, default=7)
    p_lora.add_argument("--cr-index", type=int, default=0)
    p_lora.add_argument("--sync-word", type=int, default=0x12)
    p_lora.add_argument("--tx-power", type=int, default=14)
    p_lora.add_argument("--preamble-symbols", type=int, default=8)

    # Backward-compatible alias for the old radio command.
    p_radio = sub.add_parser("radio")
    p_radio.add_argument("--node", type=int, default=CONTROL_BROADCAST_NODE_ID)
    p_radio.add_argument("--freq", type=float, default=868.0)
    p_radio.add_argument("--sf", type=int, default=7)
    p_radio.add_argument("--bw", type=int, default=0)
    p_radio.add_argument("--cr", type=int, default=5)

    # [BLE only] Live-adjust how many simultaneous TX connections the RX
    # is willing to hold. Handled entirely locally by the RX firmware
    # (see setNodeCountLimit() in main_ble_rx.cpp) — never relayed to any
    # TX board, so --node is irrelevant here and intentionally omitted.
    p_nodes = sub.add_parser("nodes")
    p_nodes.add_argument(
        "count",
        type=int,
        help="Desired simultaneous BLE connections (0..MAX_TX_NODES on the RX firmware)",
    )

    return parser


def main():
    """Build the selected command packet and send or capture it."""
    parser = build_parser()
    args = parser.parse_args()

    if args.action == "rate":
        packet = make_packet(
            CMD_SET_RATE,
            target_node_id=args.node,
            interval_ms=args.interval_ms,
        )

    elif args.action == "start":
        packet = make_packet(
            CMD_START,
            target_node_id=args.node,
        )

    elif args.action == "stop":
        packet = make_packet(
            CMD_STOP,
            target_node_id=args.node,
        )

    elif args.action == "gfsk":
        packet = make_packet(
            CMD_SET_GFSK_PARAMS,
            target_node_id=args.node,
            freq_mhz=args.freq,
            gfsk_bitrate_bps=args.bitrate,
            gfsk_preamble_bits=args.preamble_bits,
            gfsk_deviation_khz=args.deviation,
            gfsk_rx_bw_index=args.rxbw_index,
        )

    elif args.action == "lora":
        packet = make_packet(
            CMD_SET_LORA_PARAMS,
            target_node_id=args.node,
            freq_mhz=args.freq,
            lora_sf=args.sf,
            lora_bw_index=args.bw_index,
            lora_cr_index=args.cr_index,
            lora_sync_word=args.sync_word,
            lora_tx_power_dbm=args.tx_power,
            lora_preamble_symbols=args.preamble_symbols,
        )

    elif args.action == "radio":
        # Legacy LoRa-oriented command.
        packet = make_packet(
            CMD_SET_RADIO_PARAMS,
            target_node_id=args.node,
            freq_mhz=args.freq,
            lora_sf=args.sf,
            lora_bw_index=args.bw,
            lora_cr_index=args.cr,
        )

    elif args.action == "nodes":
        # target_node_id is irrelevant here — the RX intercepts and acts
        # on this locally in pollSerialForControl() before the per-link
        # relay loop, so it's left at the broadcast default. The count
        # rides in interval_ms, matching setNodeCountLimit((int)cmd.intervalMs)
        # on the firmware side.
        packet = make_packet(
            CMD_SET_NODE_COUNT,
            interval_ms=args.count,
        )

    else:
        parser.error(f"Unhandled action: {args.action}")

    if args.port:
        port = args.port
    elif args.protocol:
        port = PORTS[args.protocol]
    else:
        sys.exit("Must specify either --protocol or --port")

    print(f"Sending {len(packet)}-byte packet to {port}: {packet.hex(' ')}")
    if args.action == "start" and args.log is not None:
        capture_start(port, packet, args.log, args.protocol or "custom", args.node)
    else:
        send_packet(port, packet)


if __name__ == "__main__":
    main()