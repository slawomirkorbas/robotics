import pytest
import time
from services.quadruped_service import QuadrupedService


def test_blink_light():
    #given
    service = QuadrupedService()

    #expect
    service.blink_light()
    time.sleep(2)
    service.blink_light()

    #then
    assert(True)