import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

MAX_SPEED = 2.0
MIN_DISTANCE = 0.05
OBJ_Z_OFFSET = 0.18
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
        
        self.timer = self.create_timer(0.02, self.control_loop)

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
            cube_tf = self.tf_buffer.lookup_transform(
                'panda_link0', 'Cube', rclpy.time.Time()
            )
            hand_tf = self.tf_buffer.lookup_transform(
                'panda_link0', 'panda_hand', rclpy.time.Time()
            )
        except TransformException as ex:
            return

        dx = cube_tf.transform.translation.x - hand_tf.transform.translation.x
        dy = cube_tf.transform.translation.y - hand_tf.transform.translation.y
        # small offset not to hit the cube
        dz = (cube_tf.transform.translation.z + OBJ_Z_OFFSET) - hand_tf.transform.translation.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        msg = TwistStamped()
        msg.header.stamp = cube_tf.header.stamp
        msg.header.frame_id = 'panda_link0' 


        if distance < MIN_DISTANCE:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
        else:
            dir_x = dx / distance
            dir_y = dy / distance
            dir_z = dz / distance

            speed = min(distance * 2.5, MAX_SPEED)

            msg.twist.linear.x = dir_x * speed
            msg.twist.linear.y = dir_y * speed
            msg.twist.linear.z = dir_z * speed

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
