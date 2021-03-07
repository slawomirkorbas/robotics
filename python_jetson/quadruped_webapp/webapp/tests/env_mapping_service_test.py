

def test_scan():
    service = EnvMappingService()
    result = service.scan(10)

    assert(result.len > 0)