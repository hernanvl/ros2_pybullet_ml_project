# force rebuild
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import pybullet as p
import pybullet_data
import numpy as np
from cv_bridge import CvBridge

class PyBulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim')

        # ROS2 interfaces
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

        self.timer = self.create_timer(1.0/30.0, self.update)

        self.linear = 0.0
        self.angular = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static base_link -> camera_link once
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'camera_link'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.5  # camera 0.5 m above base, adjust if you want
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(static_tf)


        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.linear = 0.0
        self.angular = 0.0

    def cmd_vel_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z


    def cmd_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def update(self):
        # Apply velocity
        p.resetBaseVelocity(self.robot, [self.linear, 0, 0], [0, 0, self.angular])

        # Step simulation
        p.stepSimulation()

        # Publish odometry
        odom = Odometry()
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

        # Publish camera image
        img_data = p.getCameraImage(128, 128)
        width = img_data[0]
        height = img_data[1]
        rgb = img_data[2]
        img = np.reshape(rgb, (height, width, 4))[:, :, :3].astype(np.uint8)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
