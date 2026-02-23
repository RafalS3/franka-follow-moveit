import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
class CubeFollower(Node):
    def __init__(self):
        super().__init__(
            'cube_follower_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        self.traj_sub = self.create_subscription(JointTrajectory, '/panda_arm_controller/joint_trajectory', self.traj_cb, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_commands', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def traj_cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        js = JointState()
        js.header = msg.header
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = msg.joint_names
        js.position = msg.points[0].positions
        js.velocity = msg.points[0].velocities
        self.js_pub.publish(js)

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

        dx = trans.transform.translation.x
        dy = trans.transform.translation.y
        dz = trans.transform.translation.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        gain = 1.0  
        if distance < 0.10:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
        else:
            msg.twist.linear.x = dx * gain
            msg.twist.linear.y = dy * gain
            msg.twist.linear.z = dz * gain

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
