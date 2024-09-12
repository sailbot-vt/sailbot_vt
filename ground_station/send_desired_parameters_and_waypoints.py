import requests
import pandas as pd
import time
import json

TELEMETRY_SERVER_URL = 'http://3.141.26.89:8080/'

def main():
    """
    Just a little script that sends some stuff to the telemetry server
    """
    parameters_json = json.load(open("desired_parameters.json"))
    waypoints_list = json.load(open("desired_waypoints.json"))["waypoints"]
    
    print(requests.post(TELEMETRY_SERVER_URL + "autopilot_parameters/set", json={"value": parameters_json}).json())
    print(requests.post(TELEMETRY_SERVER_URL + "waypoints/set", json={"value": waypoints_list}).json())

if __name__ == "__main__": main()