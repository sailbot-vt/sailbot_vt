import json, requests, time
TELEMETRY_SERVER_URL = "http://18.191.164.84:8080/"

start_time = time.time()
while True:
    print(time.time()- start_time)
    if (time.time() - start_time) > 300:
        waypoints_list = json.load(open("desired_waypoints2.json"))["waypoints"]
        print(requests.post(TELEMETRY_SERVER_URL + "waypoints/set", json={"value": waypoints_list}).json())
        time.sleep(1)
    
    time.sleep(0.05)