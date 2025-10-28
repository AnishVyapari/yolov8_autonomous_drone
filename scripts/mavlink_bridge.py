#!/usr/bin/env python3
"""
MAVLink bridge between flight controller and companion computer/network.
- Forwards selected MAVLink messages and allows command injection safely.
"""
import argparse
import socket
import threading

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None


def bridge(fc_endpoint: str, udp_out: str):
    if mavutil is None:
        print('pymavlink not installed. pip install pymavlink')
        return 1
    print(f'Connecting to FC at {fc_endpoint}')
    master = mavutil.mavlink_connection(fc_endpoint, autoreconnect=True)
    master.wait_heartbeat()
    print('Heartbeat received')

    out_ip, out_port = udp_out.split(':')
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def recv_loop():
        while True:
            msg = master.recv_match(blocking=True)
            if not msg:
                continue
            # Forward selected high-level telemetry only
            if msg.get_type() in ['GLOBAL_POSITION_INT', 'ATTITUDE', 'SYS_STATUS']:
                payload = str(msg).encode('utf-8')
                out_sock.sendto(payload, (out_ip, int(out_port)))

    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()

    print('Bridge running. Press Ctrl+C to stop')
    try:
        while True:
            t.join(1)
    except KeyboardInterrupt:
        print('Stopping bridge')
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--fc', default='udp:127.0.0.1:14550')
    p.add_argument('--udp-out', default='127.0.0.1:14600')
    args = p.parse_args()
    return bridge(args.fc, args.udp_out)

if __name__ == '__main__':
    raise SystemExit(main())
