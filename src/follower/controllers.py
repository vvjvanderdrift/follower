import numpy as np

import rospy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from simple_sim.msg import KinematicBicycleControl


class PID:

    def __init__(self) -> None:
        print("Initializing PID controller")

        self.time_start = rospy.Time.now()
        self.time_now = rospy.Time.now()

        self.odometry_msg = Odometry()

        self.state_topic = "/kinematic_bicycle/state"
        self.reference_marker_topic = "/reference/reference_marker_msg"
        self.control_topic = "/kinematic_bicycle/control"

        rospy.Subscriber(self.state_topic, Odometry, self.control_callback)
        self.reference_marker_pub = rospy.Publisher(self.reference_marker_topic, Marker, queue_size=1)
        self.control_pub = rospy.Publisher(self.control_topic, KinematicBicycleControl, queue_size=1)
        pass



    def control_callback(self, odometry_msg):

        self.odometry_msg = odometry_msg
        self.frame_id = odometry_msg.header.frame_id


        reference = self.generate_reference()
        error = self.calculate_error(reference)
        control_input = self.calculate_control(error)

        self.publish_control(control_input)
        
        return

    def generate_reference(self):

        self.time_now = rospy.Time.now()

        time_since_start = self.time_now - self.time_start

        # reference = np.sin(time_since_start)
        reference = time_since_start.to_sec()
        self.publish_refence_marker(reference)
        print(f'Reference: {reference}')

        return reference

    def publish_refence_marker(self,reference):
        
        reference_marker_msg = Marker()
        
        reference_marker_msg.header.frame_id = self.frame_id
        reference_marker_msg.header.stamp = rospy.Time.now()
        reference_marker_msg.type = 2
        reference_marker_msg.pose.position.x = reference
        reference_marker_msg.pose.position.y = 0
        reference_marker_msg.pose.position.z = 0
        reference_marker_msg.pose.orientation.x = 0.0
        reference_marker_msg.pose.orientation.y = 0.0
        reference_marker_msg.pose.orientation.z = 0.0
        reference_marker_msg.pose.orientation.w = 1.0
        reference_marker_msg.scale.x = 0.25
        reference_marker_msg.scale.y = 0.25
        reference_marker_msg.scale.z = 0.25
        reference_marker_msg.color.a = 1.0 
        reference_marker_msg.color.r = 0.0
        reference_marker_msg.color.g = 1.0
        reference_marker_msg.color.b = 1.0

        self.reference_marker_pub.publish(reference_marker_msg)

        return


    def calculate_error(self,reference):

        state = self.odometry_msg.pose.pose.position.x
        
        error = reference - state

        return error

    def calculate_control(self, error):
        
        proportional = 1

        control_input = [proportional * error, 0]

        return control_input

    def publish_control(self, control_input):
        
        
        control_msg = KinematicBicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)

        

        return
