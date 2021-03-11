import json


def test_lidar_sample(app, client):
    res = client.get('/lidar_sample')
    assert res.status_code == 200
    expected = {'hello': 'world'}
    assert expected == json.loads(res.get_data(as_text=True))