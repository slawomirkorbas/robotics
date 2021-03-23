
import pytest
import time
from services.env_mapping_service import EnvMappingService


def test_scan():
    #given
    service = EnvMappingService()

    #when
    service.start_scanning(lidar_scan_ready_callback)
    time.sleep(5)
    service.stop_scanning()

    #then
    print(service.angle2dist)
    assert(len(service.angle2dist) > 0)


def lidar_scan_ready_callback(angle2dist):
    assert(True)


    