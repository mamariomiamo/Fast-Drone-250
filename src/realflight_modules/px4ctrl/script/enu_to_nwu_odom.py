#!/usr/bin/env python
import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist


class ENUtoNWUConverter:
    def __init__(self):
        rospy.init_node("enu_to_nwu_odom")

        self.sub = rospy.Subscriber("/mavros/local_position/odom",
                                    Odometry,
                                    self.odom_callback,
                                    queue_size=10)

        self.pub = rospy.Publisher("/odom_nwu",
                                   Odometry,
                                   queue_size=10)

        # Rotation from ENU → NWU (rotation of -90° about Z)
        self.enu_to_nwu_quat = tft.quaternion_from_euler(0, 0, -1.57079632679)

    def odom_callback(self, msg: Odometry):
        nwu_msg = Odometry()
        nwu_msg.header = msg.header
        nwu_msg.header.frame_id = "world"
        nwu_msg.child_frame_id = "world"

        # --- Position transform ENU → NWU ---
        nwu_msg.pose.pose.position.x = msg.pose.pose.position.y
        nwu_msg.pose.pose.position.y = -msg.pose.pose.position.x
        nwu_msg.pose.pose.position.z = msg.pose.pose.position.z

        # --- Orientation transform ---
        q_enu = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        q_rot = self.enu_to_nwu_quat
        q_nwu = tft.quaternion_multiply(q_rot, q_enu)
        nwu_msg.pose.pose.orientation.x = q_nwu[0]
        nwu_msg.pose.pose.orientation.y = q_nwu[1]
        nwu_msg.pose.pose.orientation.z = q_nwu[2]
        nwu_msg.pose.pose.orientation.w = q_nwu[3]

        # --- Twist transform (linear + angular) ---
        nwu_msg.twist.twist.linear.x = msg.twist.twist.linear.y
        nwu_msg.twist.twist.linear.y = -msg.twist.twist.linear.x
        nwu_msg.twist.twist.linear.z = msg.twist.twist.linear.z

        nwu_msg.twist.twist.angular.x = msg.twist.twist.angular.y
        nwu_msg.twist.twist.angular.y = -msg.twist.twist.angular.x
        nwu_msg.twist.twist.angular.z = msg.twist.twist.angular.z

        self.pub.publish(nwu_msg)


if __name__ == "__main__":
    converter = ENUtoNWUConverter()
    rospy.spin()
