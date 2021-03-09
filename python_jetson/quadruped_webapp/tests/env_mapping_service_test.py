
import pytest
import time
from services.env_mapping_service import EnvMappingService


def test_scan():
    #given
    service = EnvMappingService()
    service.stop()
    time.sleep(2)

    #when
    angle2dist = service.scan()

    #then
    print(angle2dist)
    assert(len(angle2dist) > 0)