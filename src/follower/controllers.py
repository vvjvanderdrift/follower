import numpy as np

import rospy
import tf_conversions

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from simple_sim.msg import KinematicBicycleControl

from follower import reference


class PID_Stanley:

    def __init__(self) -> None:
        print("Initializing PID controller")

        self.reference = reference.Reference()

        self.ego_state = reference.Node()
        self.steering_angle = 0.0

        self.state_topic = "/kinematic_bicycle/state"
        self.control_topic = "/kinematic_bicycle/control"

        rospy.Subscriber(self.state_topic, Odometry, self.control_callback)
        rospy.Subscriber('/movebox/front_left_steer_joint', JointState, self.steer_callback)
        self.control_pub = rospy.Publisher(self.control_topic, KinematicBicycleControl, queue_size=1)
        pass


    def control_callback(self, odometry_msg):

        self.frame_id = odometry_msg.header.frame_id
        self.update_ego_state(odometry_msg)

        goal_node = self.reference.calculate_closest_node(self.ego_state)
        self.reference.publish_node_marker(self.frame_id, goal_node)

        control_input = self.calculate_control(goal_node)

        self.publish_control(control_input)
        
        return

    def steer_callback(self, jointstate_msg):

        self.steering_angle = jointstate_msg.position[0]

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

        self.ego_state.vx = odometry_msg.twist.twist.linear.x

        return


    def calculate_control(self, goal_node):

        error_x_global = goal_node.x - self.ego_state.x
        error_y_global = goal_node.y - self.ego_state.y

        error_x_ego = error_x_global*np.cos(self.ego_state.psi) \
                    + error_y_global*np.sin(self.ego_state.psi)
        error_y_ego =-error_x_global*np.sin(self.ego_state.psi) \
                    + error_y_global*np.cos(self.ego_state.psi)
            
        error_psi = 0 

        # Longitudinal speed PID 
        # 50 k/h = 14 m/s
        speed_ref = 0 # Reference speed 
        proportional = 1
        control_speed = speed_ref + proportional * error_x_ego

        max_speed = 7
        control_speed = min(control_speed,max_speed)


        # Lateral Stanley
        gain = 1
        control_steering_angle = error_psi + np.arctan2(gain*error_y_ego,control_speed)
        
        max_steering_angle = 30 * np.pi/180
        control_steering_angle = min(control_steering_angle, max_steering_angle)
        control_steering_angle = max(control_steering_angle, -max_steering_angle)
        
        # Longitudinal acceleration PID 
        proportional = 1
        control_acceleration = proportional * (control_speed - self.ego_state.vx)

        # Steering angle rate PID 
        proportional = 1
        control_steering_rate = proportional * (control_steering_angle - self.steering_angle)

        # control_input = [speed, steering_angle]
        control_input = [control_acceleration, control_steering_rate]

        rospy.logwarn(f'Control speed: {control_speed}, acceleration: {control_acceleration}, steering angle: {control_steering_angle}')

        return control_input


    def publish_control(self, control_input):
                
        control_msg = KinematicBicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)

        return
