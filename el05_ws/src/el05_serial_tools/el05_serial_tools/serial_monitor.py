#!/usr/bin/env python3
"""Read-only EL05 AT serial monitor. No CAN frames are transmitted."""
import argparse
import binascii
import time

import serial


def extract_frames(buffer):
    frames = []
    while True:
        start = buffer.find(b'AT')
        if start < 0:
            return b'', frames
        if len(buffer) < start + 7:
            return buffer[start:], frames
        length = buffer[start + 6]
        end = start + 7 + length + 2
        if len(buffer) < end:
            return buffer[start:], frames
        if buffer[end - 2:end] == b'\r\n':
            frames.append(buffer[start:end])
            buffer = buffer[end:]
        else:
            buffer = buffer[start + 2:]


def device_id_query(master_id=0xFD):
    """Build the manual's type-0 broadcast device-ID query (target 0x7F)."""
    raw_can_id = (0x00 << 24) | (master_id << 8) | 0x7F
    serial_can_id = (raw_can_id << 3).to_bytes(4, 'big')
    return b'AT' + serial_can_id + bytes([8]) + bytes(8) + b'\r\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description='EL05 serial receive monitor (read-only)')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=921600)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--raw', action='store_true', help='also print non-frame bytes')
    parser.add_argument('--probe-id', type=int, help='send a non-motion device-ID query to motor 1..127')
    args = parser.parse_args(argv)
    print(f'Opening {args.port} at {args.baudrate} baud; receive-only for {args.duration:g}s')
    with serial.Serial(args.port, args.baudrate, timeout=0.1) as port:
        if args.probe_id is not None:
            if not 1 <= args.probe_id <= 127:
                parser.error('--probe-id must be between 1 and 127')
            port.write(device_id_query())
            port.flush()
            print(f'Sent non-motion broadcast device-ID query (requested test id={args.probe_id})')
        buffer = b''
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            chunk = port.read(port.in_waiting or 1)
            if not chunk:
                continue
            buffer += chunk
            buffer, frames = extract_frames(buffer)
            for frame in frames:
                can_id = int.from_bytes(frame[2:6], 'big')
                length = frame[6]
                payload = frame[7:7 + length]
                print(f'CAN_ID=0x{can_id:08X} DLC={length} DATA={binascii.hexlify(payload).decode()}')
            if args.raw and buffer:
                print('RAW=' + binascii.hexlify(buffer).decode())
    print('Monitor stopped; no motion or enable frame was transmitted.')


if __name__ == '__main__':
    main()
