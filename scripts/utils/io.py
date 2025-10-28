#!/usr/bin/env python3
"""
I/O utilities for reading configs and waypoint CSVs.
"""
import yaml
import csv
from dataclasses import dataclass
from typing import List

@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float


def load_yaml(path: str):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def read_waypoints_csv(path: str) -> List[Waypoint]:
    wps: List[Waypoint] = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wps.append(Waypoint(float(row['lat']), float(row['lon']), float(row['alt'])))
    return wps
