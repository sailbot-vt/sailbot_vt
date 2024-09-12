import requests
import time
import os


TELEMETRY_SERVER_URL = 'http://3.141.26.89:8080/'

print(requests.get(TELEMETRY_SERVER_URL + "boat_public_ip/get").json())
