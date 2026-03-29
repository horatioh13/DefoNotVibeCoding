import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

# Import the class we built in the last step
from sulis.sulis.grbl_controller import GrblController

class GantryHardwareBridge(Node):
    def __init__(self):
        super().__init__('gantry_hardware_bridge')
        
        # --- 1. Initialize Hardware ---
        self.get_logger().info("Connecting to GRBL...")
        self.gantry = GrblController(port='/dev/ttyUSB0')
        self.gantry.start()
        
        # Optional: Home the gantry on startup
        # self.gantry.home()
        # time.sleep(5) 

        # --- 2. ROS 2 Subscriptions ---
        # MoveIt Servo outputs velocities here
        self.servo_sub = self.create_subscription(
            JointJog,
            '/servo_node/delta_joint_cmds',
            self.servo_callback,
            10
        )
        
        # MoveIt 2 Planner outputs paths here
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/gantry_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # --- 3. ROS 2 Publishers (Joint States) ---
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states at 30Hz
        self.state_timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)
        
        # Keep track of current assumed positions (in meters for ROS)
        self.current_x_m = 0.0
        self.current_z_m = 0.0
        
        self.get_logger().info("Gantry Bridge Node Ready.")

    def servo_callback(self, msg: JointJog):
        """Handles live streaming velocity commands from a joystick via MoveIt Servo."""
        vx_mm_sec = 0.0
        vz_mm_sec = 0.0
        
        for i, joint_name in enumerate(msg.joint_names):
            # Convert ROS meters/sec to GRBL millimeters/sec
            if joint_name == 'gantry_horizontal_joint':
                vx_mm_sec = msg.velocities[i] * 1000.0 
            elif joint_name == 'gantry_vertical_joint':
                vz_mm_sec = msg.velocities[i] * 1000.0

        # Send to our background thread
        self.gantry.update_jog_velocity(vx_mm_sec, vz_mm_sec)

    def trajectory_callback(self, msg: JointTrajectory):
        """Handles calculated absolute paths from MoveIt."""
        final_point = msg.points[-1]
        
        # Grab current REAL position to use as defaults in case 
        # a joint is missing from the trajectory message
        current_x_mm, current_z_mm = self.gantry.get_current_position()
        target_x_mm = current_x_mm
        target_z_mm = current_z_mm
        
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name == 'gantry_horizontal_joint':
                target_x_mm = final_point.positions[i] * 1000.0
            elif joint_name == 'gantry_vertical_joint':
                target_z_mm = final_point.positions[i] * 1000.0
                
        self.get_logger().info(f"Executing planned move to X:{target_x_mm} Z:{target_z_mm}")
        self.gantry.move_absolute(target_x_mm, target_z_mm, speed_mm_sec=20.0)

    def publish_joint_states(self):
        """Reads the TRUE hardware position and tells MoveIt where the gantry is."""
        
        # Read the latest parsed data from the GRBL thread
        current_x_mm, current_z_mm = self.gantry.get_current_position()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gantry_horizontal_joint', 'gantry_vertical_joint']
        
        # Convert millimeters back to meters for ROS
        msg.position = [
            current_x_mm / 1000.0, 
            current_z_mm / 1000.0
        ]
        
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = GantryHardwareBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.gantry.stop()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
