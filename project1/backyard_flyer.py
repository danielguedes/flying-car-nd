import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        # self.check_state = {} # commented because it's never used
        self.gps_ready = False
        self.takeoff_altitude = 3

        self.flight_state = States.MANUAL

        self.register_callback(MsgID.GLOBAL_POSITION, self.global_position_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def global_position_callback(self):
        if self.flight_state == States.MANUAL:

            # LOCAL_POSITION message can contains weird values if it's received before
            # first GLOBAL_POSITION message (might be a bug)
            # so we wait for the first message before transition to arming
            self.gps_ready = True

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:

            # takeoff detection uses absolute value instead of percentage
            # to not loose precision in higher altitudes
            if self.in_range(-self.local_position[2], self.takeoff_altitude, 0.15):
                self.waypoint_transition()
        
        elif self.flight_state == States.WAYPOINT:

            if self.in_waypoint(self.target_position):
                # is_still provides precise navigation and landing
                if len(self.all_waypoints) < 1 and self.is_still:
                    self.landing_transition()
                elif self.is_still:
                    self.waypoint_transition()

    def velocity_callback(self):
        max_velocity = 1.0
        self.is_still = abs(self.local_velocity[0]) < max_velocity and abs(self.local_velocity[1]) < max_velocity and abs(self.local_velocity[2]) < max_velocity

        if self.flight_state == States.LANDING:
            
            # global_home[2] isn't used for landing detection
            # because it's value is always zero (looks like a bug)
            if abs(self.local_position[2]) < 0.01:
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            if self.gps_ready:
                self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        return [
            [15, 0, 3, 0],
            [15, 15, 3, 1.5708],
            [0, 15, 3, 3.14159],
            [0, 0, 3, 4.71239]
        ]

    def in_waypoint(self, waypoint):
        margin = 0.5
        position = self.local_position
        return self.in_range(position[0], waypoint[0], margin) and self.in_range(position[1], waypoint[1], margin) and self.in_range(-position[2], waypoint[2], margin)
        
    def in_range(self, a, b, margin):
        return abs(a-b) < margin

    def arming_transition(self):
        print("arming transition")

        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.take_control()
        self.arm()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")

        self.takeoff(self.takeoff_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")

        self.target_position = self.all_waypoints.pop(0)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        print("Going to Waypoint: ", self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")

        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")

        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
