import numpy as np

import rospy
import tf_conversions

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from simple_sim.msg import BicycleControl
from lmpcc_msgs.msg import Control

from follower import reference

class LocalMotionTracker:
    # This is a follower which takes the reference inputs calculated by the MPC planning module and tracks these signals to produce a high-frequency signal for the actuators

    def __init__(self, ctrl_dt=1, ego_model='kinematic_bicycle', control_input_symbols='jx_domega') -> None:

        self.frame_id = "map"

        self.ctrl_dt = ctrl_dt
        self.ego_model = ego_model
        self.control_input_symbols = control_input_symbols
        self.mpc_optimal_control_symbols = 'ax_omega'

        self.mpc_optimal_controls = Control(throttle=0.0,steer=0.0)
        self.current_state = [0.0, 0.0] # Should be same quantity (or symbols) as 'mpc_optimal_control_symbols'

        self.mpc_optimal_controls_topic = "/lmpcc"
        self.control_inputs_topic = "/" + self.ego_model + "/control"
        self.throttle_state_topic = "/value/ax"
        self.steer_state_topic = "/value/omega"

        rospy.Subscriber(self.mpc_optimal_controls_topic, Control, self.mpc_optimal_controls_callback)
        rospy.Subscriber(self.throttle_state_topic, Float32, self.throttle_state_callback)
        rospy.Subscriber(self.steer_state_topic, Float32, self.steer_state_callback)
        self.control_pub = rospy.Publisher(self.control_inputs_topic, BicycleControl, queue_size=1)

        self.ax_control_pub = rospy.Publisher('/control/ax', Float32, queue_size=1)
        self.omega_control_pub = rospy.Publisher('/control/omega', Float32, queue_size=1)

        pass

    def mpc_optimal_controls_callback(self, mpc_optimal_control_msg):

        self.mpc_optimal_controls = mpc_optimal_control_msg

        return

    def throttle_state_callback(self, throttle_state_msg):
        
        self.current_state[0] = throttle_state_msg.data

        return

    def steer_state_callback(self, steer_state_msg):
        
        self.current_state[1] = steer_state_msg.data

        return

    def timer_callback(self, event):
        
        self.control()

        return
    
    def control(self):

        reference = np.array([self.mpc_optimal_controls.throttle,
                     self.mpc_optimal_controls.steer])
        
        state = np.array(self.current_state)

        error = reference - state

        Kp = np.array([20.,20.])

        control_input = Kp*error
        control_input = control_input.tolist()

        self.publish_control(control_input)

        self.ax_control_pub.publish(Float32(reference[0]))
        self.omega_control_pub.publish(Float32(reference[1]))

        return 
    
    def publish_control(self, control_input):

        control_msg = BicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)


        return

class StanleyReferenceTracker:
    # This is a follower which takes a reference point (from e.g. a roadmap) and tracks this point using a longitudinal PID controller and lateral stanley controller

    def __init__(self, ctrl_dt=1, ego_model='kinematic_bicycle', control_input_symbols='ax_omega') -> None:

        self.ctrl_dt = ctrl_dt

        self.reference = reference.Reference()
        self.frame_id = "map"

        self.ego_model = ego_model
        self.control_input_symbols = control_input_symbols

        # Gains
        if(ego_model == "kinematic_bicycle" and self.control_input_symbols == "ax_omega"):
            self.Kp_long_speed = 3
            self.Kd_long_speed = 0
            self.K_stanley = 1
            self.Kp_long_accel = 5
            self.Kp_steer_rate = 25
            self.Kd_steer_rate = 0
        elif(ego_model == "kinematic_bicycle" and self.control_input_symbols == "vx_delta"):
            self.Kp_long_speed = 1
            self.Kd_long_speed = 0
            self.K_stanley = 1
            self.Kp_long_accel = 0
            self.Kp_steer_rate = 0
            self.Kd_steer_rate = 0
        elif(ego_model == "dynamic_bicycle"):
            # self.Kp_long_speed = 3
            # self.Kd_long_speed = 0
            # self.K_stanley = 1
            # self.Kp_long_accel = 5
            # self.Kp_steer_rate = 25
            # self.Kd_steer_rate = 0
            self.Kp_long_speed = 3
            self.Kd_long_speed = 0
            self.K_stanley = 1
            self.Kp_long_accel = 5
            self.Kp_steer_rate = 25
            self.Kd_steer_rate = 0
            
        self.ego_state = reference.Node()
        self.steering_angle = 0.0

        self.previous_error_x_ego = 0.0
        self.previous_error_steering_angle = 0.0

        self.state_topic = "/" + self.ego_model + "/state"
        self.control_inputs_topic = "/" + self.ego_model + "/control"

        rospy.Subscriber(self.state_topic, Odometry, self.odometry_callback)
        rospy.Subscriber('/movebox/front_left_steer_joint', JointState, self.steer_callback)
        self.control_pub = rospy.Publisher(self.control_inputs_topic, BicycleControl, queue_size=1)
        self.speed_value_pub = rospy.Publisher('/value/speed', Float32, queue_size=1)
        self.steering_angle_value_pub = rospy.Publisher('/value/steering_angle', Float32, queue_size=1)
        self.speed_reference_pub = rospy.Publisher('/value/speed_reference', Float32, queue_size=1)
        self.steering_angle_reference_pub = rospy.Publisher('/value/steering_angle_reference', Float32, queue_size=1)

        pass


    def odometry_callback(self, odometry_msg):

        self.frame_id = odometry_msg.header.frame_id
        self.update_ego_state(odometry_msg)

        # TODO: Add timed callback
        if(not self.ctrl_dt):
            self.control()
        
        return

    def timer_callback(self, event):
        
        self.control()

        return
        
    def steer_callback(self, jointstate_msg):

        self.steering_angle = jointstate_msg.position[0]

        return

    def control(self):

        goal_node = self.reference.calculate_closest_node(self.ego_state)
        self.reference.publish_node_marker(self.frame_id, goal_node)

        control_input = self.calculate_control(goal_node)

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

        d_error_x_ego = error_x_ego - self.previous_error_x_ego
        control_speed = speed_ref + self.Kp_long_speed * error_x_ego + self.Kd_long_speed * d_error_x_ego

        max_speed = 14
        control_speed = min(control_speed,max_speed)

        # Lateral Stanley
        control_steering_angle = error_psi + np.arctan(self.K_stanley*error_y_ego/control_speed)
        
        max_steering_angle = 30 * np.pi/180
        control_steering_angle = min(control_steering_angle, max_steering_angle)
        control_steering_angle = max(control_steering_angle, -max_steering_angle)

        if(self.control_input_symbols == "vx_delta"):
            control_input = [control_speed, control_steering_angle]

        elif(self.control_input_symbols == "ax_omega"):
            
            # Longitudinal acceleration PID 
            control_acceleration = self.Kp_long_accel * (control_speed - self.ego_state.vx)

            # Steering angle rate PID 
            error_steering_angle = (control_steering_angle - self.steering_angle)
            d_error_steering_angle = error_steering_angle - self.previous_error_steering_angle
            control_steering_rate = self.Kp_steer_rate * error_steering_angle + self.Kd_steer_rate * d_error_steering_angle

            # control_input = [speed, steering_angle]
            control_input = [control_acceleration, control_steering_rate]

        # rospy.logwarn(f'Control speed: {control_speed}, acceleration: {control_acceleration}, steering angle: {control_steering_angle}')


        self.speed_value_pub.publish(Float32(self.ego_state.vx))
        self.steering_angle_value_pub.publish(Float32(self.steering_angle * 180 / np.pi))
        self.speed_reference_pub.publish(Float32(control_speed))
        self.steering_angle_reference_pub.publish(Float32(control_steering_angle * 180 / np.pi))

        self.previous_error_x_ego = error_x_ego
        if(self.control_input_symbols == "ax_omega"):
            self.previous_error_steering_angle = error_steering_angle

        return control_input


    def publish_control(self, control_input):

        control_msg = BicycleControl()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.frame_id

        control_msg.u = control_input

        self.control_pub.publish(control_msg)

        return

