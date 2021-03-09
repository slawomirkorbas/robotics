from injector import singleton

from services.quadruped_service import QuadrupedService
from services.env_mapping_service import EnvMappingService

def configure(binder):
    binder.bind(QuadrupedService, to=QuadrupedService, scope=singleton)
    binder.bind(EnvMappingService, to=EnvMappingService, scope=singleton)
    
