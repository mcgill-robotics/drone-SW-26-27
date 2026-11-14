def pipeline(c):
    return f"nvarguscamerasrc sensor-id={c['sensor_id']} ! nvvidconv ! video/x-raw,format=I420,width={c['width']},height={c['height']}"
