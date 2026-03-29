import rclpy
from rclpy.node import Node
import rclpy.utilities

from std_msgs.msg import Bool, Float32MultiArray
from adafruit_servokit import ServoKit
import numpy as np

import serial
import time
import threading

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

servo_board_index = {
    "shoulder":15,
    "elbow":14,
    "wrist":13,
    "gripper":12,
}



class GrblController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.grbl = None
        
        # --- Shared State Variables ---
        self.target_vx = 0.0  
        self.target_vy = 0.0  
        self.priority_command = None 
        
        # --- NEW: Actual Position Tracking ---
        self.actual_x_mm = 0.0
        self.actual_y_mm = 0.0
        
        self.lock = threading.Lock()
        self.is_running = False
        self.thread = None
        self.DT = 0.05 

    def start(self):
        # Open the port with control lines disabled from the start
        self.grbl = serial.Serial()
        self.grbl.port = self.port
        self.grbl.baudrate = self.baudrate
        self.grbl.timeout = 0.1
        self.grbl.dsrdtr = False
        self.grbl.rtscts = False
        self.grbl.dtr = False
        self.grbl.rts = False
        self.grbl.open()
        
        # Wake up GRBL
        self.grbl.write(b'\r\n\r\n')
        time.sleep(2)
        self.grbl.reset_input_buffer()
        
        # --- NEW: Enforce GRBL Configuration ---
        if self.verbose:
            print("Enforcing GRBL Settings...")
        

        # Keep steppers constantly energized (Holding Torque)
        self.grbl.write(b'$1=255\n') 
        time.sleep(0.1)
        self.grbl.write(b'$X\n') 
        time.sleep(0.1)

        
        # Report Machine Position (MPos) in the '?' status string
        self.grbl.write(b'$10=1\n')  
        time.sleep(0.1)
        
        # Clear out the 'ok' responses from the config commands
        self.grbl.reset_input_buffer() 
        
        self.is_running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        if self.verbose:
            print("GRBL Control Thread Started (Holding Torque ON, MPos Polling ON).")

    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join()
        if self.grbl and self.grbl.is_open:
            self.grbl.write(b'\x85') 
            time.sleep(0.1)
            self.grbl.close()

    # ==========================================
    #             USER / ROS INTERFACE
    # ==========================================

    def update_jog_velocity(self, vx_mm_sec, y_mm_sec):
        with self.lock:
            self.target_vx = vx_mm_sec
            self.target_vy = y_mm_sec

    def home(self):
        with self.lock:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.priority_command = "$H"

    def move_absolute(self, x_mm, y_mm, speed_mm_sec):
        feedrate = speed_mm_sec * 60.0 
        with self.lock:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.priority_command = f"G90\nG1 X{x_mm:.3f} Y{y_mm:.3f} F{feedrate:.1f}"

    def get_current_position(self):
        """NEW: Safely retrieves the latest hardware position."""
        with self.lock:
            return self.actual_x_mm, self.actual_y_mm

    # ==========================================
    #             BACKGROUND THREAD
    # ==========================================

    def _parse_status_line(self, line):
        """NEW: Extracts MPos from strings like <Idle|MPos:15.0,0.0,-5.0|FS:0,0>"""
        if line.startswith('<') and 'MPos:' in line:
            try:
                # Isolate the MPos:xxx,yyy,zzz block
                mpos_str = line.split('MPos:')[1].split('|')[0].split('>')[0]
                parts = mpos_str.split(',')
                
                # GRBL outputs X, Y, Z. We are using X and Z for the gantry.
                x = float(parts[0])
                y = float(parts[1]) 
                
                with self.lock:
                    self.actual_x_mm = x
                    self.actual_y_mm = y
            except Exception as e:
                pass # Ignore malformed serial lines

    def _wait_for_ok(self):
        """Blocks until GRBL finishes the current command, but still parses status."""
        while self.is_running:
            line = self.grbl.readline().decode('utf-8', errors='ignore').strip()
            if line == 'ok':
                break
            elif line.startswith('<'):
                self._parse_status_line(line)
            elif 'error:8' in line:
                if self.verbose:
                    print(f"GRBL Response: {line} (command parse error, retrying)")
                time.sleep(0.05)
                break
            elif 'error:15' in line:
                if self.verbose:
                    print(f"GRBL Response: {line} (clearing alarm with $X)")
                # Clear alarm and retry
                self.grbl.write(b'$X\n')
                time.sleep(0.1)
                break
            elif 'error' in line or 'ALARM' in line:
                if self.verbose:
                    print(f"GRBL Response: {line}")
                break

    def _control_loop(self):
        was_jogging = False
        last_poll_time = time.time()
        
        while self.is_running:
            # 1. Grab current target commands
            with self.lock:
                vx = self.target_vx
                vy = self.target_vy
                p_cmd = self.priority_command
                self.priority_command = None 
            
            is_jogging_now = abs(vx) > 0.1 or abs(vy) > 0.1
            
            # --- Poll GRBL for status at 10Hz, but NOT during active jogging ---
            if not is_jogging_now:
                current_time = time.time()
                if current_time - last_poll_time > 0.1:
                    self.grbl.write(b'?') # The real-time query character
                    last_poll_time = current_time
            
            # --- Read any pending serial data (like our status report) ---
            while self.grbl.in_waiting > 0:
                line = self.grbl.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('<'):
                    self._parse_status_line(line)
                
            # 2. Handle Priority Commands
            if p_cmd:
                if was_jogging:
                    self.grbl.write(b'\x85') 
                    time.sleep(0.2)
                    self.grbl.reset_input_buffer()
                    was_jogging = False
                
                for line in p_cmd.split('\n'):
                    self.grbl.write((line + '\n').encode('utf-8'))
                    self._wait_for_ok() 
                    
            # 3. Handle Live Jogging
            elif abs(vx) > 0.1 or abs(vy) > 0.1:
                step_x = vx * self.DT
                step_y = vy * self.DT
                feedrate = max(abs(vx), abs(vy)) * 60.0
                feedrate = max(1.0, feedrate)  # Enforce minimum feedrate
                
                cmd = f"$J=G91 X{step_x:.3f} Y{step_y:.3f} Z0.000 F{feedrate:.1f}\n"
                if self.verbose:
                    print(f"[DEBUG] Jog command: {cmd.strip()}")  # Debug output
                self.grbl.write(cmd.encode('utf-8'))
                self.grbl.flush()  # Ensure command is sent immediately
                time.sleep(0.01)  # Give GRBL time to process
                self._wait_for_ok()
                was_jogging = True
                
            # 4. Handle Stopping/Idle
            else:
                if was_jogging:
                    self.grbl.write(b'\x85') 
                    time.sleep(0.2)
                    self.grbl.reset_input_buffer()
                    was_jogging = False
                
                time.sleep(0.01)

class Actuator(Node):
    def __init__(self):
        super().__init__("actuator")
        
        self.position_subscriber_ = self.create_subscription(
                Float32MultiArray, 
                "/EffectorPositions", 
                self.set_targets, 
                10
        )

        self.job_position_controller_ = self.create_subscription(
                Float32MultiArray,
                "/Gantry",
                self.gantry,
                10
        )

        self.home_sub_ = self.create_subscription(Bool, "/Home", self.home, 10)

        self.target_effector_array = [0] * 4
        self.target_jog_array = [0] * 4
        
        self.kit_ = ServoKit(channels=16)

        self.gerbl = GrblController()
        self.gerbl.start()

        # timer of 10ms for driving the motors
        # self.create_timer(0.01, self.drive)

    # Stepper x (1), Stepper y (2), Velocity x (3), Velocity y (4), Shoulder (5), Elbow (6), Wrist (7) and Gripper (8)
    # Shoulder (5), Elbow (6), Wrist (7) and Gripper (8)
    def set_targets(self, msg):
        self.target_effector_array = msg.data
        self.get_logger().info(str(msg.data))

        shoulder = self.bound_180(self.target_effector_array[control_index["shoulder"]])
        elbow = self.bound_180(self.target_effector_array[control_index["elbow"]])
        wrist = self.bound_180(self.target_effector_array[control_index["wrist"]])
        gripper = self.bound_180(self.target_effector_array[control_index["gripper"]])

        self.kit_.servo[servo_board_index["shoulder"]].angle = shoulder
        self.kit_.servo[servo_board_index["elbow"]].angle = elbow
        self.kit_.servo[servo_board_index["wrist"]].angle = wrist
        self.kit_.servo[servo_board_index["gripper"]].angle = gripper

    def home(self, msg):
        if msg.data:
            self.gerbl.home()

    def gantry(self, msg):
        # msg.data x, y pos
        self.gerbl.move_absolute(-msg.data[0], -msg.data[1], 50)

    def bound_180(self, value):
        if value > 180:
            value = 180
        elif value < 0:
            value = 0
        return value


def degrees_to_rad(val):
    return (np.pi / 180) * val

def main(args=None):
    rclpy.init(args=args)

    actuator = Actuator()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(actuator)
    try:
        rclpy.spin(actuator)
    except KeyboardInterrupt:
        actuator.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        actuator.destroy_node()
        rclpy.utilities.try_shutdown()

