import requests
import base64, json, numpy as np, cv2

TELEMETRY_SERVER_URL = 'http://3.141.26.89:8080/'

def main():
    base64_encoded_image = requests.get(TELEMETRY_SERVER_URL + "/boat_status/get").json()["current_camera_image"]
    
    jpg_original = base64.b64decode(base64_encoded_image)

    jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
    img = cv2.imdecode(jpg_as_np, flags=1)
    cv2.imwrite('test.jpg', img)



    

if __name__ == "__main__":
    main()