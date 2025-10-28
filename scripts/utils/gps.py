#!/usr/bin/env python3
"""
GPS utilities: helpers to compute distances/bearings and simple geofence check.
"""
import math

R_EARTH = 6371000.0

def haversine(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dl/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R_EARTH * c

def bearing(lat1, lon1, lat2, lon2):
    y = math.sin(math.radians(lon2-lon1))*math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1))*math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(math.radians(lon2-lon1))
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360) % 360

def inside_circle(lat, lon, center_lat, center_lon, radius_m):
    return haversine(lat, lon, center_lat, center_lon) <= radius_m
