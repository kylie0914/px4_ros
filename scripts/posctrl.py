#!/usr/bin/env python
# ROS python API
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, ExtendedState, State
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler


class OffboardCtrl:
    def __init__(self):

        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.local_position = PoseStamped()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'local_pos', 'state']
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")



        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)




        # Publisher
        self.desired_pos = PoseStamped()
        self.desired_pos.pose.position.x = 0
        self.desired_pos.pose.position.y = 0
        self.desired_pos.pose.position.z = 0
        self.radius = 1.0
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def send_pos(self):
        rate = rospy.Rate(20)  # Hz
        self.desired_pos.header = Header()
        self.desired_pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.desired_pos.header.stamp = rospy.Time.now()

            self.pos_setpoint_pub.publish(self.desired_pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


    def halt(self):
        self.desired_pos.pose.position.x = self.local_position.pose.position.x
        self.desired_pos.pose.position.y = self.local_position.pose.position.y
        self.desired_pos.pose.position.z = self.local_position.pose.position.z
        


    def altitude_callback(self, data):
        self.altitude = data
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True


    def global_position_callback(self, data):
        self.global_position = data
        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True


    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True


    def state_callback(self, data):
        self.state = data
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def extended_state_callback(self, data):
        self.extended_state = data
        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True


    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)


    def is_at_position(self, d_pos, offset):
        """offset: meters"""
        desired = np.array(d_pos[0:3])
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))

        return np.linalg.norm(desired - pos) < offset


    def move_to(self, d_pos):
        """timeout(int): seconds"""
        # set a position setpoint
        self.desired_pos.pose.position.x = d_pos[0]
        self.desired_pos.pose.position.y = d_pos[1]
        self.desired_pos.pose.position.z = d_pos[2]

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = d_pos[3]  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.desired_pos.pose.orientation = Quaternion(*quaternion)


    def reach_position(self, d_pos, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.desired_pos.pose.position.x = d_pos[0]
        self.desired_pos.pose.position.y = d_pos[1]
        self.desired_pos.pose.position.z = d_pos[2]
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = d_pos[3]  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.desired_pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(d_pos, self.radius):
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)



def main():

    rospy.init_node('setpoint_node', anonymous=True)

    print("node started")
    agent = OffboardCtrl()
    print("Class loaded")
    agent.wait_for_topics(20)
    print("Topic loaded")


    pos_takeoff = [0, 0, 2, 0]
    pos0 = [0, 0, 2, 45]
    pos1 = [4, -2, 2, 45]
    pos_land = [0, 0, 0, 0]
    loop_rate = rospy.Rate(10)

    print("Test trajectories")

    #agent.set_mode_srv(custom_mode='OFFBOARD')
    #agent.set_arming_srv(True)

    #agent.reach_position(pos_takeoff, 30)
    
    agent.halt()
    print("Set desired as current position - Init")
    
    agent.move_to(pos_takeoff)
    while(agent.is_at_position(pos_takeoff, agent.radius)==False):
        loop_rate.sleep()
    print("Take off Done")

    # 10 sec hovering
    hov_time = 30  # 10 Hz update
    for _ in range(hov_time):
        loop_rate.sleep()
        
    agent.move_to(pos0)
    while(agent.is_at_position(pos0, agent.radius)==False):
        loop_rate.sleep()
        
    for _ in range(hov_time):
        loop_rate.sleep()

    agent.move_to(pos1)
    while(agent.is_at_position(pos1, agent.radius)==False):
        loop_rate.sleep()
    
    for _ in range(hov_time):
        loop_rate.sleep()


    landing_condition = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    agent.set_mode_srv(custom_mode="AUTO.LAND")
    while(agent.extended_state.landed_state != landing_condition):
        loop_rate.sleep()

    print("Landing done")

    agent.set_arming_srv(False)



if __name__ == '__main__':
    main()
