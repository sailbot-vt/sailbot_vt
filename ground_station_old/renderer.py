from utils import *
from typing import List
import cv2
import numpy as np
from pydantic.utils import deep_update


BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (200, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 120, 0)
CYAN = (0, 255, 255)


class RendererState():
    def __init__(self, state: State):
        # heading angle
        self.theta_boat = state["theta_boat"][2]
        self.dt_theta_boat = np.abs(state["dt_theta_boat"][2]) * angle_to_vec(self.theta_boat + np.sign(state["dt_theta_boat"][2]) * np.pi / 2)  # is either -90 or 90 degrees

        # position
        self.p_boat = np.array([state["p_boat"][0], state["p_boat"][1]])
        self.dt_p_boat = 6 * np.array([state["dt_p_boat"][0], state["dt_p_boat"][1]])

        # rudder
        self.theta_rudder = np.pi + self.theta_boat + state["theta_rudder"][0]
        self.dt_rudder = np.abs(state["dt_theta_rudder"][0]) * angle_to_vec(self.theta_rudder + np.sign(state["dt_theta_rudder"][0]) * np.pi / 2)  # is either -90 or 90 degrees

        # sail
        self.theta_sail = np.pi + self.theta_boat + state["theta_sail"][0]
        self.dt_sail = np.abs(state["dt_theta_sail"][0]) * angle_to_vec(self.theta_sail + np.sign(state["dt_theta_sail"][0]) * np.pi / 2)  # is either -90 or 90 degrees

        # wind
        self.apparent_wind = state["apparent_wind"]
        self.true_wind = state["true_wind"]

        # water
        self.water = state["water"]
        
        self.cur_waypoint_index = state["cur_waypoint_index"]
        self.waypoints = state["waypoints"]
        
        self.buoys = state["buoys"]
        
        self.no_go_zone_size = state["no_go_zone_size"]
        self.decision_zone_size = state["decision_zone_size"]


class CV2DRenderer():
    def __init__(self, size=512, padding=30, vector_scale=0.25, style={}):
        self.size = size
        self.padding = padding
        self.vector_scale = vector_scale
        self.map_bounds = None
        self.center = None
        
        self.trail_positions = []

        self.style = {
            "background": WHITE,
            "border": {
                "color": rgba(BLACK, .2),
                "width": 1,
            },
            "boat": {
                "color": rgba(BLACK, .3),
                "spike_coef": 2,
                "size": 10,
                "phi": np.deg2rad(40),
                "dt_p": {
                    "color": GREEN,
                    "width": 1,
                },
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
                "center": {
                    "color": WHITE,
                    "radius": 2,
                }
            },
            "rudder": {
                "color": rgba(BLACK, .3),
                "width": 2,
                "height": 5,
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
            },
            "sail": {
                "color": rgba(BLACK, .7),
                "width": 2,
                "height": 15,
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
            },
            "wind": {
                "color": rgba(BLUE, .5),
                "width": 2,
            },
            "water": {
                "color": rgba(CYAN, .5),
                "width": 2,
            },
        }
        self.style = deep_update(self.style, style)

    def _create_empty_img(self):
        bg = np.array(self.style["background"])
        img = bg[None, None, :] + np.zeros((self.size, self.size, 3))
        return img.astype(np.uint8)

    def _scale_to_fit_in_img(self, x):
        return x / (self.map_bounds[1] - self.map_bounds[0]).max() * (self.size - 2 * self.padding)

    def _translate_and_scale_to_fit_in_map(self, x):
        return self._scale_to_fit_in_img(x - self.map_bounds[0]) + self.padding

    def _transform_state_to_fit_in_img(self, state: RendererState):
        # translate and scale positions
        state.p_boat = self._translate_and_scale_to_fit_in_map(state.p_boat)

        # scale vectors
        # state.dt_p_boat = self._scale_to_fit_in_img(state.dt_p_boat)
        state.dt_theta_boat = self._scale_to_fit_in_img(state.dt_theta_boat)
        state.dt_rudder = self._scale_to_fit_in_img(state.dt_rudder)
        state.dt_sail = self._scale_to_fit_in_img(state.dt_sail)
        state.true_wind = self._scale_to_fit_in_img(state.true_wind)
        state.water = self._scale_to_fit_in_img(state.water)


    def _draw_borders(self, img: np.ndarray):
        borders = self._translate_and_scale_to_fit_in_map(self.map_bounds).astype(int)
        cv2.rectangle(
            img=img,
            pt1=tuple(borders[0]),
            pt2=tuple(borders[1]),
            color=self.style["border"]["color"],
            thickness=self.style["border"]["width"],
            lineType=cv2.LINE_AA
        )


    def _draw_true_wind(self, img: np.ndarray, state: RendererState):
        img_center = np.array([30, 420])
        
        cv2.arrowedLine(
            img=img,
            pt1=tuple(img_center.astype(int)),
            pt2=tuple((img_center + 40 * state.true_wind * self.vector_scale).astype(int)),
            color=self.style["wind"]["color"],
            thickness=self.style["wind"]["width"],
            tipLength=0.2,
            line_type=cv2.LINE_AA
        )


    def _draw_water(self, img: np.ndarray, state: RendererState):
        img_center = np.array([self.size, self.size]) / 2
        
        cv2.arrowedLine(
            img=img,
            pt1=tuple(img_center.astype(int)),
            pt2=tuple((img_center + state.water * self.vector_scale).astype(int)),
            color=self.style["water"]["color"],
            thickness=self.style["water"]["width"],
            tipLength=0.2,
            line_type=cv2.LINE_AA
        )


    def _draw_no_go_zone_lines(self, img: np.ndarray, state: RendererState):
        line_start = np.array([420, 420])
        line_end1 = line_start + 40 * rotate_vector(np.array([0, 1]), np.deg2rad(180 - state.no_go_zone_size/2))
        line_end2 = line_start + 40 * rotate_vector(np.array([0, 1]), np.deg2rad(180 + state.no_go_zone_size/2))
        
        cv2.line(
            img=img,
            pt1=tuple(line_start.astype(int)),
            pt2=tuple(line_end1.astype(int)),
            color=(139, 0, 0),
            thickness=1,
            lineType=cv2.LINE_AA
        )
        
        cv2.line(
            img=img,
            pt1=tuple(line_start.astype(int)),
            pt2=tuple(line_end2.astype(int)),
            color=(139, 0, 0),
            thickness=1,
            lineType=cv2.LINE_AA
        )
        
        
        
        if np.linalg.norm(state.apparent_wind) != 0:
            normalized_apparent_wind = state.apparent_wind/ np.linalg.norm(state.apparent_wind)
            
            line_start = state.p_boat
            line_end1 = line_start + 40 * rotate_vector(normalized_apparent_wind, np.deg2rad(180 - state.no_go_zone_size/2))
            line_end2 = line_start + 40 * rotate_vector(normalized_apparent_wind, np.deg2rad(180 + state.no_go_zone_size/2))
            
            cv2.line(
                img=img,
                pt1=tuple(line_start.astype(int)),
                pt2=tuple(line_end1.astype(int)),
                color=(0, 0, 139),
                thickness=1,
                lineType=cv2.LINE_AA
            )
            
            cv2.line(
                img=img,
                pt1=tuple(line_start.astype(int)),
                pt2=tuple(line_end2.astype(int)),
                color=(0, 0, 139),
                thickness=1,
                lineType=cv2.LINE_AA
            )
            
            
    def _draw_decision_zone_lines(self, img: np.ndarray, state: RendererState):
        
        line_start = np.array([420, 420])
        line_end1 = line_start + 40 * rotate_vector(np.array([0, 1]), np.deg2rad(180 - state.decision_zone_size/2))
        line_end2 = line_start + 40 * rotate_vector(np.array([0, 1]), np.deg2rad(180 + state.decision_zone_size/2))
        
        cv2.line(
            img=img,
            pt1=tuple(line_start.astype(int)),
            pt2=tuple(line_end1.astype(int)),
            color=(139, 0, 0),
            thickness=1,
            lineType=cv2.LINE_AA
        )
        
        cv2.line(
            img=img,
            pt1=tuple(line_start.astype(int)),
            pt2=tuple(line_end2.astype(int)),
            color=(139, 0, 0),
            thickness=1,
            lineType=cv2.LINE_AA
        )
        
        if np.linalg.norm(state.true_wind) != 0:
            normalized_true_wind = state.true_wind/ np.linalg.norm(state.true_wind)
            
            line_start = state.p_boat
            line_end1 = line_start + 40 * rotate_vector(normalized_true_wind, np.deg2rad(np.deg2rad(state.theta_boat) + 180 - state.decision_zone_size/2))
            line_end2 = line_start + 40 * rotate_vector(normalized_true_wind, np.deg2rad(np.deg2rad(state.theta_boat) + 180 + state.decision_zone_size/2))
            
            cv2.line(
                img=img,
                pt1=tuple(line_start.astype(int)),
                pt2=tuple(line_end1.astype(int)),
                color=RED,
                thickness=1,
                lineType=cv2.LINE_AA
            )
            
            cv2.line(
                img=img,
                pt1=tuple(line_start.astype(int)),
                pt2=tuple(line_end2.astype(int)),
                color=RED,
                thickness=1,
                lineType=cv2.LINE_AA
            )
        
        
    def _draw_boat(self, img: np.ndarray, state: RendererState):
        boat_size = self.style["boat"]["size"]
        phi = self.style["boat"]["phi"]
        spike_coeff = self.style["boat"]["spike_coef"]
        sailboat_pts = np.array([
            [state.p_boat + angle_to_vec(state.theta_boat + phi) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat + (np.pi - phi)) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat + (np.pi + phi)) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat - phi) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat) * spike_coeff * boat_size]
        ], dtype=int)
        
        cv2.fillConvexPoly(
            img=img,
            points=sailboat_pts,
            color=self.style["boat"]["color"],
            lineType=cv2.LINE_AA
        )

    def _draw_trail(self, img: np.ndarray, state: RendererState):
        self.trail_positions.append(state.p_boat)
        for trail_dot_pos in self.trail_positions:
            cv2.circle(
                img=img,
                center=tuple(trail_dot_pos.astype(int)),
                radius=1,
                color=(125, 125, 125),
                thickness=-1
            )
        
        
    def _draw_desired_heading_line(self, img: np.ndarray, state: RendererState, next_waypoint: tuple):
        line_start = state.p_boat
        line_end = next_waypoint
        
        cv2.line(
            img=img,
            pt1=tuple(line_start.astype(int)),
            pt2=tuple(line_end.astype(int)),
            color=BLACK,
            thickness=1,
            lineType=cv2.LINE_AA
        )
        
        
        
    def _draw_apparent_wind_angle(self, img: np.ndarray, state: RendererState):
        arrow_start = np.array((420, 420))
        apparent_wind_speed = np.linalg.norm(state.apparent_wind)
    
        _, AWA = cartesian_vector_to_polar(state.apparent_wind[0], state.apparent_wind[1])
        AWA -= np.rad2deg(state.theta_boat)
        AWA -= 90
        
        arrow_end = (int(arrow_start[0] + 100 * apparent_wind_speed * np.cos(np.deg2rad(AWA))), int(arrow_start[1] - 100 * apparent_wind_speed * np.sin(np.deg2rad(AWA))))
        cv2.arrowedLine(
            img=img,
            pt1=arrow_start,
            pt2=arrow_end,
            color=self.style["wind"]["color"],
            thickness=self.style["wind"]["width"],
            tipLength=0.2,
            line_type=cv2.LINE_AA
        )

        
    def _draw_sail(self, img: np.ndarray, state: RendererState):
        sail_height = self.style["sail"]["height"]
        sail_start = state.p_boat
        sail_end = sail_start + angle_to_vec(state.theta_sail) * sail_height
        
        cv2.line(
            img=img,
            pt1=tuple(sail_start.astype(int)),
            pt2=tuple(sail_end.astype(int)),
            color=self.style["sail"]["color"],
            thickness=self.style["sail"]["width"],
            lineType=cv2.LINE_AA
        )


    def _draw_rudder(self, img: np.ndarray, state: RendererState):
        rudder_height = self.style["rudder"]["height"]
        boat_phi = self.style["boat"]["phi"]
        boat_size = self.style["boat"]["size"]
        back_of_boat = state.p_boat + angle_to_vec(np.pi + state.theta_boat) * np.cos(boat_phi) * boat_size
        rudder_start = back_of_boat
        rudder_end = rudder_start + angle_to_vec(state.theta_rudder) * rudder_height
            
        cv2.line(
            img=img,
            pt1=tuple(rudder_start.astype(int)),
            pt2=tuple(rudder_end.astype(int)),
            color=self.style["rudder"]["color"],
            thickness=self.style["rudder"]["width"],
            lineType=cv2.LINE_AA
        )


    def _draw_boat_pos_velocity(self, img: np.ndarray, state: RendererState):
        dt_p_boat_start = state.p_boat
        dt_p_boat_end = dt_p_boat_start + 3 * state.dt_p_boat * self.vector_scale
        
        cv2.arrowedLine(
            img=img,
            pt1=tuple(dt_p_boat_start.astype(int)),
            pt2=tuple(dt_p_boat_end.astype(int)),
            color=self.style["boat"]["dt_p"]["color"],
            thickness=self.style["boat"]["dt_p"]["width"],
            tipLength=.2,
            line_type=cv2.LINE_AA
        )
        

    def _draw_boat_heading_velocity(self, img: np.ndarray, state: RendererState):
        spike_coeff = self.style["boat"]["spike_coef"]
        boat_size = self.style["boat"]["size"]
        front_of_boat = state.p_boat + angle_to_vec(state.theta_boat) * spike_coeff * boat_size

        dt_theta_boat_start = front_of_boat
        dt_theta_boat_end = dt_theta_boat_start + state.dt_theta_boat * self.vector_scale
        
        cv2.arrowedLine(
            img=img,
            pt1=tuple(dt_theta_boat_start.astype(int)),
            pt2=tuple(dt_theta_boat_end.astype(int)),
            color=self.style["boat"]["dt_theta"]["color"],
            thickness=self.style["boat"]["dt_theta"]["width"],
            tipLength=.2,
            line_type=cv2.LINE_AA
        )


    def _draw_rudder_velocity(self, img: np.ndarray, state: RendererState):
        boat_phi = self.style["boat"]["phi"]
        boat_size = self.style["boat"]["size"]
        rudder_height = self.style["rudder"]["height"]
        back_of_boat = state.p_boat + angle_to_vec(np.pi + state.theta_boat) * np.cos(boat_phi) * boat_size
        dt_rudder_start = back_of_boat + angle_to_vec(state.theta_rudder) * rudder_height
        dt_rudder_end = dt_rudder_start + state.dt_rudder
        
        cv2.arrowedLine(
            img=img,
            pt1=tuple(dt_rudder_start.astype(int)),
            pt2=tuple(dt_rudder_end.astype(int)),
            color=self.style["rudder"]["dt_theta"]["color"],
            thickness=self.style["rudder"]["dt_theta"]["width"],
            tipLength=.2,
            line_type=cv2.LINE_AA
        )
        

    def _draw_sail_velocity(self, img: np.ndarray, state: RendererState):
        sail_height = self.style["sail"]["height"]
        dt_sail_start = state.p_boat + angle_to_vec(state.theta_sail) * sail_height
        dt_sail_end = dt_sail_start + state.dt_sail
        
        cv2.arrowedLine(
            img=img,
            pt1=tuple(dt_sail_start.astype(int)),
            pt2=tuple(dt_sail_end.astype(int)),
            color=self.style["sail"]["dt_theta"]["color"],
            thickness=self.style["sail"]["dt_theta"]["width"],
            tipLength=.2,
            line_type=cv2.LINE_AA
        )
        

    def _draw_boat_center(self, img: np.ndarray, state: RendererState):
        cv2.circle(
            img=img,
            center=tuple(state.p_boat.astype(int)),
            radius=self.style["boat"]["center"]["radius"],
            color=self.style["boat"]["center"]["color"],
            thickness=-1
        )

    def _draw_waypoint(self, img: np.ndarray, x_position, y_position, color):
        cv2.circle(
            img=img, 
            center=(int(x_position), int(y_position)), 
            radius=5, 
            color=color, 
            thickness=-1
        )
        
        
    def _draw_buoys(self, img: np.ndarray, buoys):
        for buoy in buoys:
            cv2.circle(
                img=img, 
                center=(int(buoy[0]), int(buoy[1])), 
                radius=5, 
                color=(0,165,255), 
                thickness=-1
            )
    
        
    def get_render_mode(self) -> str:
        return 'rgb_array'

    def get_render_modes(self) -> List[str]:
        return ['rgb_array']

    def setup(self, map_bounds):
        self.map_bounds = map_bounds[:, 0:2]  # ignore z axis
        self.center = (self.map_bounds[0] + self.map_bounds[1]) / 2


    def render(self, state, draw_extra_fct=None):
        """
        Args:
            state (State): A State object (from utils.py) that represents the boat's current state (stuff like position, speed, wind, etc).
            local_waypoints (list[tuples]): A list of all of the desired waypoints to draw in local coordinates.

        Returns:
            np.ndarray: the image to be displayed
        """
        
        assert (self.map_bounds is not None
                and self.center is not None), "Please call setup() first."

        img = self._create_empty_img()

        # prepare state
        state = RendererState(state)
        self._transform_state_to_fit_in_img(state)
        waypoints = [self._translate_and_scale_to_fit_in_map(np.array(waypoint)) for waypoint in state.waypoints]
        buoys = [self._translate_and_scale_to_fit_in_map(np.array(buoy)) for buoy in state.buoys]

        # draw extra stuff
        if draw_extra_fct is not None:
            draw_extra_fct(img, state)

        # draw map
        self._draw_trail(img, state)
        
        if waypoints:
            self._draw_desired_heading_line(img, state, waypoints[state.cur_waypoint_index])
        
        self._draw_borders(img)
        self._draw_water(img, state)
        self._draw_decision_zone_lines(img, state)
        self._draw_no_go_zone_lines(img, state)
        self._draw_boat(img, state)
        self._draw_apparent_wind_angle(img, state)
        self._draw_boat_heading_velocity(img, state)
        self._draw_boat_pos_velocity(img, state)
        self._draw_rudder(img, state)
        self._draw_rudder_velocity(img, state)
        self._draw_sail(img, state)
        self._draw_sail_velocity(img, state)
        self._draw_boat_center(img, state)
        self._draw_true_wind(img, state)
        self._draw_buoys(img, buoys)
        
        if waypoints:
            for index, (x, y) in enumerate(waypoints):
                if index == state.cur_waypoint_index:
                    continue
                else:
                    self._draw_waypoint(img, x, y, (0, 0, 0))
            
            x,y = waypoints[state.cur_waypoint_index]
            self._draw_waypoint(img, x, y, (255, 0, 0))
        
        # flip vertically
        img = img[::-1, :, :]

        return img
