#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_py

pose = PoseStamped()
is_sub = bool()

class Pose2odom(Node):

    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    def __init__(self):
        super().__init__('pose_to_odom')
        self.subscription = self.create_subscription(
            PoseStamped,
            'map2base_pose',
            self.pose_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(Odometry, 'gnss_odom', 10)
        timer_period = 0.02 # seconds. means 50 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # x = 0.
        # y = 0.
        self.dt = 1./50.
        is_sub = False

    def timer_callback(self):
        # self.publisher_.publish(msg)
        # self.i += 1
        # rate = rospy.Rate(50.0)

        while rclpy.ok():
            (v_roll, v_pitch, v_yaw) = self.quaternion_to_euler_angle(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)
            #(v_roll, v_pitch, v_yaw) = self.quaternion_to_euler_angle()
            v_phi = float((v_roll))
            v_theta = float((v_pitch))
            v_psi = float((v_yaw))

            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z

            yaw = math.radians(v_psi)

            if self.i > 0:
                vel_x_world = (x - x_prev) / self.dt
                vel_y_world = (y - y_prev) / self.dt

                x_prev = x
                y_prev = y

                twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
                twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world

                odom = Odometry()
                odom.header.frame_id = 'world'
                odom.child_frame_id = 'gnss/base_link'
                odom.header.stamp = self.get_clock().now().to_msg()

                odom.pose.pose.position.x = pose.pose.position.x
                odom.pose.pose.position.y = pose.pose.position.y
                odom.pose.pose.position.z = pose.pose.position.z

                odom.pose.pose.orientation.x = pose.pose.orientation.x
                odom.pose.pose.orientation.y = pose.pose.orientation.y
                odom.pose.pose.orientation.z = pose.pose.orientation.z
                odom.pose.pose.orientation.w = pose.pose.orientation.w

                odom.twist.twist.linear.x = twist_x
                odom.twist.twist.linear.y = twist_y
                odom.twist.twist.linear.z = (z - z_prev) / self.dt
                z_prev = z

                odom.twist.twist.angular.x = 0.
                odom.twist.twist.angular.y = 0.
                odom.twist.twist.angular.z = 0.

                global is_sub

                if is_sub == True:
                    self.publisher_.publish(odom)
                    # br = tf.TransformBroadcaster()
                    # br.sendTransform((x, y, z), [pose.pose.orientation.x, pose.pose.orientation.y,
                    #                          pose.pose.orientation.z, pose.pose.orientation.w], rospy.Time.now(), "gnss/base_link", "map")
                    is_sub = False

            else:
                x_prev = x
                y_prev = y
                z_prev = z
                self.i += 1

    def pose_callback(self, msg):
        global pose
        pose = msg
        global is_sub
        is_sub = True

def main(args=None):
    rclpy.init(args=args)

    pose_to_odom = Pose2odom()

    rclpy.spin(pose_to_odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_to_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
