import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class CubeFollower(Node):
    def __init__(self):
        super().__init__('cube_follower_node')
        
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'panda_hand',  # parent
                'Cube',      # child
                rclpy.time.Time() 
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Waiting for tf to be published... {ex}', throttle_duration_sec=1.0)
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_hand'

        gain = 1.5
        
        msg.twist.linear.x = trans.transform.translation.x * gain
        msg.twist.linear.y = trans.transform.translation.y * gain
        msg.twist.linear.z = trans.transform.translation.z * gain
        
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

def main():
    try:
        rclpy.init()
        node = CubeFollower()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
