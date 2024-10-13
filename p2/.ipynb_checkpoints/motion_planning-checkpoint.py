import argparse
import time
import msgpack
from enum import Enum, auto
from typing import List, Tuple

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if (not self.armed) & (not self.guided):
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        relative_file_path = './colliders.csv'

        # The first line of the.csv file contains the necessary data.
        the_first_colliders_data = np.genfromtxt(relative_file_path, delimiter=',', max_rows=1, dtype=None)

        # Given the data's format, I extract the first and second elements.
        # Since each item takes the pattern xxx0_ (for example, lat0_),
        # I extract starting with the sixth character and transform it to a numeric value.
        lat0 = np.array([the_first_colliders_data[0][5:]],dtype='float64')
        lon0 = np.array([the_first_colliders_data[1][5:]],dtype='float64')

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        
        # Retrieve current global position
        global_position = [
            self._longitude,
            self._latitude,
            self._altitude
        ]
        
        # Convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(
            self.global_home,
            self.global_position,
            self.local_position)
        )
        # Read in obstacle map
        data = np.loadtxt('./colliders.csv', delimiter=',', dtype='float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # Convert start position to current position rather than map center
        grid_start = (
            int(local_position[0] - north_offset),
            int(local_position[1] - east_offset)
        )
        print("The grid start is: {}".format(grid_start))
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)

        # Adapt to set goal as latitude / longitude position and convert
        # lon_goal = -122.400511  # TODO: To be changed -122.401300
        # lat_goal = 37.792582  # TODO: To be changed 37.796750
        lon_goal = -122.401300  # TODO: To be changed -122.401300
        lat_goal = 37.796750  # TODO: To be changed 37.796750
        alt_goal = TARGET_ALTITUDE
        global_goal = [
            lon_goal,
            lat_goal, 
            alt_goal
        ]
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (
            int(local_goal[0]) - north_offset,
            int(local_goal[1]) - east_offset
        )
        # Run A* to find a path from start to goal
        # Add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # Prune path to minimize number of waypoints        
        pruned_path = self.prune_path(path)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()


    def prune_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Prunes unnecessary waypoints from a given path by checking if consecutive points 
        are collinear. If three points are nearly collinear, the middle point is skipped.
    
        Parameters:
        -----------
        path : List[Tuple[float, float]]
            A list of 2D points representing a path. Each point is a tuple (x, y).
    
        Returns:
        --------
        List[Tuple[float, float]]
            A new pruned path with unnecessary intermediate points removed.
        """
    
        def to_homogeneous(point: Tuple[float, float]) -> np.ndarray:
            """Converts a 2D point to a 3x1 homogeneous coordinate."""
            return np.array([point[0], point[1], 1.0]).reshape(1, -1)
    
        def are_points_collinear(
            p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, epsilon: float = 0.01
        ) -> bool:
            """
            Checks if three points are collinear using the determinant method.
    
            Parameters:
            -----------
            p1, p2, p3 : np.ndarray
                Homogeneous coordinates of the three points.
    
            epsilon : float, optional (default=0.01)
                A small tolerance value for checking collinearity. If the absolute value 
                of the determinant is less than epsilon, the points are considered collinear.
    
            Returns:
            --------
            bool
                True if the points are collinear, False otherwise.
            """
            matrix = np.concatenate((p1, p2, p3), axis=0)
            determinant = np.linalg.det(matrix)
            return abs(determinant) < epsilon
    
        # Initialize the pruned path with the first point.
        pruned_path = [path[0]]
    
        # Track the last two points being considered for pruning.
        previous_point = path[0]
        current_point = path[1]
    
        # Iterate through the remaining points in the path.
        for i in range(2, len(path)):
            next_point = path[i]
    
            # Check if the three points are collinear.
            if are_points_collinear(
                to_homogeneous(previous_point),
                to_homogeneous(current_point),
                to_homogeneous(next_point)
            ):
                # If collinear, skip the current point.
                current_point = next_point
            else:
                # If not collinear, add the current point to the pruned path.
                pruned_path.append(current_point)
                previous_point = current_point
                current_point = next_point
    
        # Add the last point to the pruned path.
        pruned_path.append(path[-1])
    
        return pruned_path

    
    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
