import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import airsim
from dataclasses import dataclass, asdict
import json


@dataclass
class GpsData:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0

    def __str__(self):
        return f"GpsData(latitude={self.latitude}, longitude={self.longitude}, altitude={self.altitude})"
    
    def __repr__(self):
        return f"GpsData(latitude={self.latitude}, longitude={self.longitude}, altitude={self.altitude})"


@dataclass
class GimbalData:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    def __str__(self):
        return f"GimbalData(pitch={self.pitch}, yaw={self.yaw}, roll={self.roll})"
    
    def __repr__(self):
        return f"GimbalData(pitch={self.pitch}, yaw={self.yaw}, roll={self.roll})"
    
@dataclass
class SnapshotData:
    timestamp: float = 0.0
    imagePath: str = ""
    image_type: int = 0
    width: int = 0
    height: int = 0
    image_data_uint8: bytes = b""

    def __str__(self):
        return f"SnapshotData(type={self.image_type}, width={self.width}, height={self.height}, data_size={len(self.image_data_uint8)})"
    
    def __repr__(self):
        return (f"SnapshotData(timestamp={self.timestamp}, imagePath='{self.imagePath}', "
                f"image_type={self.image_type}, width={self.width}, height={self.height}, "
                f"data_size={len(self.image_data_uint8)})")

@dataclass
class MagnetometerData:
    time_stamp: int = 0
    magnetic_field_body: tuple = (0.0, 0.0, 0.0)
    magnetic_field_covariance: float = 0.0

    def __str__(self):
        return f"MagnetometerData(time_stamp={self.time_stamp}, magnetic_field_body={self.magnetic_field_body}, magnetic_field_covariance={self.magnetic_field_covariance})"
    
    def __repr__(self):
        return f"MagnetometerData(time_stamp={self.time_stamp}, magnetic_field_body={self.magnetic_field_body}, magnetic_field_covariance={self.magnetic_field_covariance})"    

@dataclass
class DroneData:
    local_position_data: tuple = (0.0, 0.0, 0.0)
    gps_data: GpsData = GpsData()
    gimbal_data: GimbalData = GimbalData()
    snapshot_data_list: list[SnapshotData] = None
    magnetometer_data: MagnetometerData = MagnetometerData()

    def __str__(self):
        return f"DroneData(gps_data={self.gps_data}, gimbal_data={self.gimbal_data}, snapshot_data_list={self.snapshot_data_list}, magnetometer_data={self.magnetometer_data})"

    def __len__(self):
        # Count non-None fields (gps_data, gimbal_data, snapshot_data_list)
        count = 0
        if getattr(self, "gps_data", None) is not None:
            count += 1
        if getattr(self, "gimbal_data", None) is not None:
            count += 1
        if getattr(self, "snapshot_data_list", None) is not None:
            count += 1
        return count

    def __repr__(self):
        return (
            f"DroneData(\n"
            f"  gps_data={repr(self.gps_data)},\n"
            f"  gimbal_data={repr(self.gimbal_data)},\n"
            f"  snapshot_data_list={repr(self.snapshot_data_list)}\n"
            f")"
        )
