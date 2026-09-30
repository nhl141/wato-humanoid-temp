import time, rclpy, joint_config as jc
from common_msgs.msg import MotorFeedback

m=jc.load_joint_map(jc.installed_joint_command_config())