import numpy as np

import rospy
import tf_conversions

from nav_msgs.msg import Odometry
from simple_sim.msg import KinematicBicycleControl

from follower import reference


class PID_Stanley:

    def __init__(self) -> None:
        print("Initializing PID controller")

        self.reference = reference.Reference()

        self.ego_state = reference.Node()

        self.state_topic = "/kinematic_bicycle/state"
        self.control_topic = "/kinematic_bicycle/control"

        rospy.Subscriber(self.state_topic, Odometry, self.control_callback)
        self.control_pub = rospy.Publisher(self.control_topic, KinematicBicycleControl, queue_size=1)
        pass


    def control_callback(self, odometry_msg):

        self.frame_id = odometry_msg.header.frame_id
        self.update_ego_state(odometry_msg)

        closest_node = self.reference.calculate_closest_node(self.ego_state)
        self.reference.publish_node_marker(self.frame_id, closest_node)

        control_input = self.calculate_control(closest_node)

        self.publish_control(control_input)
        
        return


    def update_ego_state(self,odometry_msg):

        self.ego_state.x = odometry_msg.pose.pose.position.x
        self.ego_state.y = odometry_msg.pose.pose.position.y
        q = [odometry_msg.pose.pose.orientation.x,\
             odometry_msg.pose.pose.orientation.y,\
             odometry_msg.pose.pose.orientation.z,\
             odometry_msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf_conversions.transformations.euler_from_quaternion(q)

        self.ego_state.psi = yaw

        return


    def calculate_control(self, closest_node):

        error_x_global = closest_node.x - self.ego_state.x
        error_y_global = closest_node.y - self.ego_state.y

        error_x_ego = error_x_global*np.cos(self.ego_state.psi) \
                    + error_y_global*np.sin(self.ego_state.psi)
        error_y_ego =-error_x_global*np.sin(self.ego_state.psi) \
                    + error_y_global*np.cos(self.ego_state.psi)
            
        error_psi = 0 

        # Longitudinal PID 
        # 50 k/h = 14 m/s
        speed_ref = 0 # Reference speed 
        proportional = 5
        speed = speed_ref + proportional * error_x_ego

        max_speed = 14
        speed = min(speed,max_speed)

        # Lateral Stanley
        gain = 3
        steering_angle = error_psi + np.arctan2(gain*error_y_ego,speed)
        
        max_steering_angle = 30 * np.pi/180
        steering_angle = min(steering_angle, max_steering_angle)
        steering_angle = max(steering_angle, -max_steering_angle)
        
        control_input = [speed, steering_angle]

        return control_input


    def publish_control(self, control_input):
                
        control_msg = KinematicBicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)

        return
