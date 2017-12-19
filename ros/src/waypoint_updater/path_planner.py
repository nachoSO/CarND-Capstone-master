import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from copy import deepcopy

def get_plane_distance(target1, target2):
    delta_x = target1.pose.position.x - target2.pose.position.x
    delta_y = target1.pose.position.y - target2.pose.position.y
    return math.sqrt(delta_x * delta_x + delta_y * delta_y)

class PathPlanner(object):
    def __init__(self, lookahead_wps):
        self.current_pose = None
        self.base_waypoints = None
        self.lookahead_wps = lookahead_wps
        # vehicle initial and next index into base_waypoints
        self.init_index = None
        self.next_index = None

    def set_base_waypoints(self, waypoints):
        self.base_waypoints = waypoints
    
    def update_vehicle_location(self, curr_pose):
        self.current_pose = curr_pose
        if self.init_index is None:
            self.init_index = self.find_closest_waypoint_index(0, curr_pose)

    def find_closest_waypoint_index(self, start_index, curr_pose):
        if self.base_waypoints is None:
            return None

        search_dist = 5.0
        wp_candidates = []
        for i in range(start_index, len(self.base_waypoints)):
            if get_plane_distance(self.base_waypoints[i].pose, curr_pose) > search_dist:
                continue
            wp_candidates.append(i)

        min_dist = float('inf')
        min_index = -1
        for j in range(len(wp_candidates)):
            d = get_plane_distance(self.base_waypoints[wp_candidates[j]].pose, curr_pose)
            if d < min_dist:
                min_dist = d
                min_index = wp_candidates[j]
        return min_index

    def generate_waypoints(self):
        waypoints = []
        if self.current_pose is None or self.base_waypoints is None:
            return waypoints

        if self.next_index is None:
            self.next_index = 0

        self.next_index = self.find_closest_waypoint_index(self.next_index, self.current_pose)

        cur_x = self.current_pose.pose.position.x
        cur_y = self.current_pose.pose.position.y
        next_x = self.base_waypoints[self.next_index].pose.pose.position.x
        rospy.loginfo('#__ cur_x: %f, index: %d, next_x: %f', cur_x, self.next_index, next_x)

        # hard-coded to test simulator
        speed = 16.0
        for i in range(self.next_index, self.next_index + self.lookahead_wps):
            p = deepcopy(self.base_waypoints[i])
            p.twist.twist.linear.x = speed

            waypoints.append(p)

        rospy.loginfo('### generated # of waypoints: %d', len(waypoints))
        return waypoints

