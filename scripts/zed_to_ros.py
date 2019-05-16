import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from threading import Thread


# ZED topic name
# /zed/zed_node/odom -> nav_msgs/Odometry
# pose -> geometry_msgs/PoseWithCovariance

