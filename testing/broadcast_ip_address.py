import requests
import time
import os


TELEMETRY_SERVER_URL = 'http://3.141.26.89:8080/'


def main():
    # while True:
    #     ip_address = str(os.popen('hostname -I').read()).split(" ")[0]
    #     requests.post(TELEMETRY_SERVER_URL + "api/wp/one", json={"w": {"ip": ip_address}}).text
    #     time.sleep(0.2)
    print(requests.get(TELEMETRY_SERVER_URL + "boat_public_ip/get").json()["ip"])


if __name__ == "__main__":
    main()