import json
from services.quadruped_service import QuadrupedService
from services.env_mapping_service import EnvMappingService

def test_lidar_sample(app, client):
    res = client.get('/lidar_sample')
    assert res.status_code == 200
    expected = {'hello': 'world'}
    assert expected == json.loads(res.get_data(as_text=True))


def test_collision_detections():
    #given
    q_service = QuadrupedService()
    env_service = EnvMappingService()
    #env_service.stop_scanning()

    #when
    q_service.free_walk_with_collision_avoidance(env_service)

    #then
    assert(True)
