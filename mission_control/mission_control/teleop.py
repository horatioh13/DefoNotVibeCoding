import rclpy
from rclpy.node import Node
import rclpy.utilities

from control_msgs.msg import JointJog
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from mission_control.config.mappings import AXES, BUTTONS 

control_index = {
    # "stepper-pos-x": 0, 
    # "stepper-pos-y": 1, 
    # "stepper-velocity-x": 2, 
    # "stepper-velocity-y":3,
    "shoulder":0,
    "elbow":1,
    "wrist":2,
    "gripper":3,
}

jog_index = {
    "x": 0,
    "y": 1,
}

EFFECTOR_CONTROL_LENGTH = 4

class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        
        node_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.target_effector_array = [0.0] * 4
        self.target_joint_jog = [0.0] * 2
        
        self.position_effector_publisher_ = self.create_publisher(
                Float32MultiArray, 
                "/EffectorPositions", 
                10
        )

        self.jog_publisher_ = self.create_publisher(
                JointJog, 
                '/servo_node/delta_joint_cmds',
                10
        )

        self.controller_commands_sub_ = self.create_subscription(
            Joy,
            "/joy",
            self.teleopCB_,
            qos_profile=qos_profile_sensor_data,
            callback_group=node_cb_group,
        )

        self.joint_set = 0
        self.accel_scale = 0.18
        self.gantry_scale = 15 / 1000

        # timer of 10ms for driving the motors
        self.create_timer(0.1, self.set_target_effector)


    def teleopCB_(self, msg):
        if msg.buttons[BUTTONS["TRIANGLE"]]:
            self.joint_set = control_index["shoulder"]
        elif msg.buttons[BUTTONS["CIRCLE"]]:
            self.joint_set = control_index["elbow"]
        elif msg.buttons[BUTTONS["CROSS"]]:
            self.joint_set = control_index["wrist"]
        elif msg.buttons[BUTTONS["SQUARE"]]:
            self.joint_set = control_index["gripper"]

        self.target_effector_array[self.joint_set] += ((msg.axes[AXES["TRIGGERRIGHT"]]*-1) + 1)  * self.accel_scale
        self.target_effector_array[self.joint_set] -= ((msg.axes[AXES["TRIGGERLEFT"]]*-1) + 1)  * self.accel_scale

        self.target_effector_array[self.joint_set] = self.bound_180(self.target_effector_array[self.joint_set])

        self.target_joint_jog[0] = msg.axes[AXES["LEFTX"]] * self.gantry_scale 
        self.target_joint_jog[1] = msg.axes[AXES["LEFTY"]] * self.gantry_scale * 0.5 

    def set_target_effector(self):
        msg = Float32MultiArray()
        self.get_logger().info(str(self.target_effector_array))
        msg.data = [float(self.target_effector_array[0]), float(self.target_effector_array[1]),
            float(self.target_effector_array[2]), float(self.target_effector_array[3])]
        # msg.layout = MultiArrayLayout()
        # msg.layout.dim = EFFECTOR_CONTROL_LENGTH

        self.position_effector_publisher_.publish(msg)
        
        msg = JointJog()
        msg.joint_names = ["gantry_horizontal_joint", "gantry_vertical_joint"]
        msg.velocities = [float(self.target_joint_jog[0]), float(self.target_joint_jog[1])]
        msg.duration = 1.0

        self.jog_publisher_.publish(msg)

    def bound_180(self, value):
        if value > 180:
            value = 180
        elif value < -180:
            value = -180
        return value



def main(args=None):
    rclpy.init(args=args)

    teleop = Teleop()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(actuator)
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        teleop.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        teleop.destroy_node()
        rclpy.utilities.try_shutdown()

