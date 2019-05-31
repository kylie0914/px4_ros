#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
from pymavlink import mavutil
from offboard_comm import *


def main():

    rospy.init_node('setpoint_node', anonymous=True)

    print("node started")
    agent = OffboardCtrl()
    print("Class loaded")
    agent.wait_for_topics(20)
    print("Topic loaded")


    # pose
    # Yaw, X, Y, Z
    pos_takeoff = [0, 0, 0, 2]
    pos1 = [0, -5, 3, 2]
    pos2 = [0, 1, 1, -0.5]
    pos3 = [0, -3, -2, 1.5]
    loop_rate = rospy.Rate(10)

    print("Test trajectories")

    #agent.set_mode_srv(custom_mode='OFFBOARD')
    #agent.set_arming_srv(True)

    #agent.reach_position(pos_takeoff, 30)
    
    agent.halt()
    print("Set desired as current position - Init")

    # 3 sec hovering
    hov_time = 30  # 10 Hz update


    agent.move_to(pos_takeoff)
    while(agent.is_at_position(pos_takeoff, agent.radius)==False):
        loop_rate.sleep()
    print("Take off Done")

    for _ in range(hov_time):
        loop_rate.sleep()


    #agent.dronet_mv(pos_takeoff)
    agent.dronet_mv(pos1)
    agent.dronet_mv(pos2)
    agent.dronet_mv(pos3)



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
