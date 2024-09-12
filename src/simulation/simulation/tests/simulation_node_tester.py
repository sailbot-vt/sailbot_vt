from simulation.simulation_node import SimNode
import rclpy
from geometry_msgs.msg import Vector3
import numpy as np

ALLOWED_FLOAT_ERROR = 0.1

def main():
    rclpy.init()
    sim_node = SimNode()

    mag, dir = sim_node.calculate_magnitude_and_direction_from_vector(Vector3(x=0.9659258262890683, y=0.25881904510252074, z=0.))
    print(dir)
    assert abs(dir - 15) <= ALLOWED_FLOAT_ERROR

    mag, dir = sim_node.calculate_magnitude_and_direction_from_vector(Vector3(x=-0.9659258262890683, y=0.25881904510252074, z=0.))
    print(dir)
    assert abs(dir - 165) <= ALLOWED_FLOAT_ERROR

    mag, dir = sim_node.calculate_magnitude_and_direction_from_vector(Vector3(x=-0.9659258262890683, y=-0.25881904510252074, z=0.))
    print(dir)
    assert abs(dir - 195) <= ALLOWED_FLOAT_ERROR

    mag, dir = sim_node.calculate_magnitude_and_direction_from_vector(Vector3(x=0.9659258262890683, y=-0.25881904510252074, z=0.))
    print(dir)
    assert abs(dir - 345) <= ALLOWED_FLOAT_ERROR

    print("Simulation node test has run successfully!!")

    sim_node.__del__()


if __name__ == "__main__": main()