from injector import singleton

from quadruped_service import QuadrupedService

def configure(binder):
    binder.bind(QuadrupedService, to=QuadrupedService, scope=singleton)
