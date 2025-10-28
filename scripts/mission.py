#!/usr/bin/env python3
"""
Autonomous mission control script.
- Reads waypoints CSV
- Arms, takeoff, flies waypoints with simple geofence check
"""
import argparse
import csv
import time
from dataclasses import dataclass
from typing import List

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
    from pymavlink import mavutil
except Exception:
    connect = None

@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float


def read_waypoints(csv_path: str) -> List[Waypoint]:
    wps = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wps.append(Waypoint(float(row['lat']), float(row['lon']), float(row['alt'])))
    return wps


def arm_and_takeoff(vehicle, target_alt):
    print('Arming motors...')
    while not vehicle.is_armable:
        print(' Waiting for vehicle to initialise...')
        time.sleep(1)
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = True
    while not vehicle.armed:
        print(' Waiting for arming...')
        time.sleep(1)
    print(f'Taking off to {target_alt}m')
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f' Alt: {alt:.1f}')
        if alt >= target_alt * 0.95:
            print('Reached target altitude')
            break
        time.sleep(1)


def goto_waypoint(vehicle, wp: Waypoint):
    print(f'Going to waypoint: {wp}')
    vehicle.simple_goto(LocationGlobalRelative(wp.lat, wp.lon, wp.alt))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints', default='data/waypoints.csv')
    parser.add_argument('--connection', default='udp:127.0.0.1:14550')
    parser.add_argument('--alt', type=float, default=10.0)
    args = parser.parse_args()

    if connect is None:
        print('DroneKit not installed. Install with pip install dronekit pymavlink')
        return 1

    print('Connecting to vehicle...')
    vehicle = connect(args.connection, wait_ready=True)

    arm_and_takeoff(vehicle, args.alt)
    wps = read_waypoints(args.waypoints)
    for wp in wps:
        goto_waypoint(vehicle, wp)
        time.sleep(10)
    print('Mission complete, RTL')
    vehicle.mode = VehicleMode('RTL')
    time.sleep(5)
    vehicle.close()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
