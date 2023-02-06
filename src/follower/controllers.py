import numpy as np

import rospy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from simple_sim.msg import KinematicBicycleControl

# class State:

#     def __init__(self):
#         # rospy.logdebug("State_init")
        


class Reference():

    def __init__(self):
        # rospy.logdebug("Reference_init")
        self.time_start = rospy.Time.now()
        self.time_now = rospy.Time.now()


        self.x = 0
        self.y = 0
        self.psi = 0


        self.reference_marker_topic = "/reference/reference_marker_msg"
        self.reference_marker_pub = rospy.Publisher(self.reference_marker_topic, Marker, queue_size=1)


    def generate(self):

        # reference = Reference()

        self.time_now = rospy.Time.now()

        time_since_start = self.time_now - self.time_start
        t = time_since_start.to_sec()

        self.x = t #(1 + np.cos(t)) * np.exp(t**.2)
        self.y = 0
        self.psi = 0


        # self.publish_marker()
        print(f'Reference(x,y,psi): {self.x, self.y, self.psi}')

        return 

    def publish_marker(self, frame_id):
        
        reference_marker_msg = Marker()
        
        reference_marker_msg.header.frame_id = frame_id
        reference_marker_msg.header.stamp = rospy.Time.now()
        reference_marker_msg.type = 2
        reference_marker_msg.pose.position.x = self.x
        reference_marker_msg.pose.position.y = self.y
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



class PID:

    def __init__(self) -> None:
        print("Initializing PID controller")

        self.odometry_msg = Odometry()

        self.reference = Reference()

        self.state_topic = "/kinematic_bicycle/state"
        self.control_topic = "/kinematic_bicycle/control"

        rospy.Subscriber(self.state_topic, Odometry, self.control_callback)
        self.control_pub = rospy.Publisher(self.control_topic, KinematicBicycleControl, queue_size=1)
        pass


    def control_callback(self, odometry_msg):

        self.odometry_msg = odometry_msg
        self.frame_id = odometry_msg.header.frame_id

        # reference = Reference()
        self.reference.generate()
        self.reference.publish_marker(self.frame_id)

        error = self.calculate_error(self.reference)
        control_input = self.calculate_control(error)

        self.publish_control(control_input)
        
        return


    def calculate_error(self, reference):


        state = self.odometry_msg.pose.pose.position.x
        
        error = reference.x - state

        return error

    def calculate_control(self, error):

        
        proportional = 1

        control_input = [proportional * error, 0.0]

        return control_input

    def publish_control(self, control_input):
        
        
        control_msg = KinematicBicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)

        
        return
