import json, requests
TELEMETRY_SERVER_URL = "http://18.191.164.84:8080/"


while True:
    waypoints_list = json.load(open("desired_waypoints.json"))["waypoints"]
    print(requests.post(TELEMETRY_SERVER_URL + "waypoints/set", json={"value": waypoints_list}).json())