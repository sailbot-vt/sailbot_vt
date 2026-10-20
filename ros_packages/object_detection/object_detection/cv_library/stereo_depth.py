import numpy as np
from scipy.optimize import linear_sum_assignment

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"

CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

class Point:
    def __init__(self):
        self.x_position
        self.y_position
        self.width
        self.height
        self.object_id
        self.class_id

class StereoMatcher():
    def __init__(self):
        self.weights = [1, 5, 2]
        self.buffer_y = 5
        self.cam_list = self._read_camera_config()

    def process_stereo(self, objs_left, objs_right):
        row_ind, col_ind = _get_matches(objs_left, objs_right)

    def _get_cost(self, left, right):
        if (left.class_id != right.class_id):
            return np.inf
        
        x_diff = left.x_position - right.x_position

        if (x_diff < 0):
            return np.inf
        
        y_diff = right.y_position - left.y_position
        if (abs(y_diff) >= self.buffer_y):
            return np.inf
        
        w_x, w_y, w_h = self.weights
        h_diff = left.height - right.height
        
        return (w_x * abs(x_diff)) + (w_y * abs(y_diff) + (w_h * abs(h_diff)))

    def _match_points(self, left, right):
        cost_matrix = np.empty((len(left), len(right)))
        # Cost matrix
        # Left points make rows
        # Right points make columns
        for i in range(len(left)):
            for j in range(len(right)):
                cost_matrix[i][j] = self._get_cost(left[i], right[j])

        return linear_sum_assignment(cost_matrix)
    
    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)
