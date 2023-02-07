import numpy as np

import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from simple_sim.msg import KinematicBicycleControl


class Node():

    def __init__(self, x=0, y=0, psi=0):

        self.x = x
        self.y = y
        self.psi = psi

        pass


    def update_pose(self, Pose):

        self.x = Pose.Point.x
        self.y = Pose.Point.y
        self.psi = 0 # TODO: Yaw not defined yet

        return


class Reference():

    def __init__(self):
        
        self.time_start = rospy.Time.now()
        self.time_now = rospy.Time.now()

        self.path = Path()

        self.reference_marker_msg = Marker()
        
        self.reference_marker_topic = "/reference/reference_marker"
        self.reference_marker_pub = rospy.Publisher(self.reference_marker_topic, Marker, queue_size=1)

        pass


    def generate_closest_node(self):

        self.time_now = rospy.Time.now()

        time_since_start = self.time_now - self.time_start
        t = time_since_start.to_sec()

        # self.x = t**1.4 #(1 + np.cos(t)) * np.exp(t**.2)
        # self.y = t**.5*np.sin(t)
        # self.psi = 0
        closest_node = Node(2*t,.5*np.sin(t),0)

        # self.closest_node.x = 2*t
        # self.closest_node.y = .5*np.sin(t)
        # self.closest_node.psi = 0

        return closest_node

    def calculate_closest_node(self, ego_state):
        
        path = []
        for x in np.arange(1000):
            path.append(Node(x*1,np.sin(x),0))

        look_over_distance = 2

        closest_node = Node(np.inf,np.inf,0)
        distances = []
        for node in path:
            dx = node.x-ego_state.x
            dy = node.y-ego_state.y
            dx_ego = dx*np.cos(ego_state.psi) + dy*np.sin(ego_state.psi)
            dy_ego = -dx*np.sin(ego_state.psi) + dy*np.cos(ego_state.psi)
            distance_node = np.hypot(dx, dy)
            distance_closest_node = np.hypot(closest_node.x-ego_state.x,closest_node.y-ego_state.y)
            
            node_in_front = dx_ego > look_over_distance
            node_closer_than_closest = distance_node < distance_closest_node
            if node_in_front and node_closer_than_closest:
                closest_node = node

        return closest_node

    def publish_node_marker(self, frame_id, node):
        
        self.reference_marker_msg.header.frame_id = frame_id
        self.reference_marker_msg.header.stamp = rospy.Time.now()
        self.reference_marker_msg.type = 2
        self.reference_marker_msg.pose.position.x = node.x
        self.reference_marker_msg.pose.position.y = node.y
        self.reference_marker_msg.pose.position.z = 0
        self.reference_marker_msg.pose.orientation.x = 0.0
        self.reference_marker_msg.pose.orientation.y = 0.0
        self.reference_marker_msg.pose.orientation.z = 0.0
        self.reference_marker_msg.pose.orientation.w = 1.0
        self.reference_marker_msg.scale.x = 0.25
        self.reference_marker_msg.scale.y = 0.25
        self.reference_marker_msg.scale.z = 0.25
        self.reference_marker_msg.color.a = 1.0 
        self.reference_marker_msg.color.r = 0.0
        self.reference_marker_msg.color.g = 1.0
        self.reference_marker_msg.color.b = 1.0

        self.reference_marker_pub.publish(self.reference_marker_msg)

        return


class PID:

    def __init__(self) -> None:
        print("Initializing PID controller")

        self.reference = Reference()

        self.ego_state = Node()

        self.state_topic = "/kinematic_bicycle/state"
        self.control_topic = "/kinematic_bicycle/control"

        rospy.Subscriber(self.state_topic, Odometry, self.control_callback)
        self.control_pub = rospy.Publisher(self.control_topic, KinematicBicycleControl, queue_size=1)
        pass


    def control_callback(self, odometry_msg):

        self.frame_id = odometry_msg.header.frame_id
        self.update_ego_state(odometry_msg)

        # closest_node = self.reference.generate_closest_node()
        closest_node = self.reference.calculate_closest_node(self.ego_state)
        self.reference.publish_node_marker(self.frame_id, closest_node)

        control_input = self.calculate_control(closest_node)

        self.publish_control(control_input)
        
        return


    def update_ego_state(self,odometry_msg):

        self.ego_state.x = odometry_msg.pose.pose.position.x
        self.ego_state.y = odometry_msg.pose.pose.position.y

        return

    def calculate_control(self, closest_node):

        error_x = closest_node.x - self.ego_state.x
        error_y = closest_node.y - self.ego_state.y
        error_psi = 0 

        # Longitudinal PID
        speed_ref = 8 / 3.6 # Reference speed 
        proportional = 5
        speed = speed_ref + proportional * error_x

        max_speed = 10 / 3.6
        speed = min(speed,max_speed)

        # Lateral Stanley
        gain = 2
        steering_angle = error_psi + np.arctan2(gain*error_y,speed)
        
        control_input = [speed, steering_angle]

        return control_input

    def publish_control(self, control_input):
        
        
        control_msg = KinematicBicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)

        
        return
