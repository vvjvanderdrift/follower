import numpy as np

import rospy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class Node():

    def __init__(self, x=0, y=0, psi=0, vx = 0):

        self.x = x
        self.y = y
        self.psi = psi
        self.vx = vx

        pass


class Reference():

    def __init__(self):
        
        self.time_start = rospy.Time.now()
        self.time_now = rospy.Time.now()

        self.path = []
        self.previous_closest_node = Node()

        self.reference_marker_msg = Marker()
        
        self.roadmap_topic = "/roadmap/reference"
        rospy.Subscriber(self.roadmap_topic, Path, self.roadmap_callback)
        self.reference_marker_topic = "/reference/reference_marker"
        self.reference_marker_pub = rospy.Publisher(self.reference_marker_topic, Marker, queue_size=1)

        pass

    def roadmap_callback(self, roadmap_reference_msg):

        path = []

        poses = roadmap_reference_msg.poses

        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            path.append(Node(x,y,0))

        self.path = path

        return 

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
                
        if self.path:
            path = self.path
        else:
            path = self.generate_path()

        closest_node = Node(np.inf,np.inf,0)
        found_node = False

        look_over_time = 1.5
        look_over_distance = max(5 , ego_state.vx * look_over_time)


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
                found_node = True

        if(found_node): 
            self.previous_closest_node = closest_node
            return closest_node
        else: 
            return self.previous_closest_node


    def generate_path(self):
        
        path = []

        # path.append(Node(0,0,0))
        
        for x in np.arange(1000):
            path.append(Node(x*1,np.sin(x),0))
        
        return path

    def publish_node_marker(self, frame_id, node):
        
        self.reference_marker_msg.header.frame_id = frame_id
        self.reference_marker_msg.header.stamp = rospy.Time.now()
        self.reference_marker_msg.type = 2
        self.reference_marker_msg.pose.position.x = node.x
        self.reference_marker_msg.pose.position.y = node.y
        self.reference_marker_msg.pose.position.z = 1
        self.reference_marker_msg.pose.orientation.x = 0.0
        self.reference_marker_msg.pose.orientation.y = 0.0
        self.reference_marker_msg.pose.orientation.z = 0.0
        self.reference_marker_msg.pose.orientation.w = 1.0
        self.reference_marker_msg.scale.x = 1.0
        self.reference_marker_msg.scale.y = 1.0
        self.reference_marker_msg.scale.z = 1.0
        self.reference_marker_msg.color.a = 1.0 
        self.reference_marker_msg.color.r = 0.0
        self.reference_marker_msg.color.g = 1.0
        self.reference_marker_msg.color.b = 1.0

        self.reference_marker_pub.publish(self.reference_marker_msg)

        return

