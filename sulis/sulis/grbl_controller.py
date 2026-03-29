import serial
import time
import threading

class GrblController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.grbl = None
        
        # --- Shared State Variables ---
        self.target_vx = 0.0  
        self.target_vz = 0.0  
        self.priority_command = None 
        
        # --- NEW: Actual Position Tracking ---
        self.actual_x_mm = 0.0
        self.actual_z_mm = 0.0
        
        self.lock = threading.Lock()
        self.is_running = False
        self.thread = None
        self.DT = 0.05 

    def start(self):
        self.grbl = serial.Serial(self.port, self.baudrate, timeout=0.1)
        
        # Wake up GRBL
        self.grbl.write(b'\r\n\r\n')
        time.sleep(2)
        self.grbl.reset_input_buffer()
        
        # --- NEW: Enforce GRBL Configuration ---
        print("Enforcing GRBL Settings...")
        
        # Keep steppers constantly energized (Holding Torque)
        self.grbl.write(b'$1=255\n') 
        time.sleep(0.1)
        
        # Report Machine Position (MPos) in the '?' status string
        self.grbl.write(b'$10=1\n')  
        time.sleep(0.1)
        
        # Clear out the 'ok' responses from the config commands
        self.grbl.reset_input_buffer() 
        
        self.is_running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
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

    def update_jog_velocity(self, vx_mm_sec, vz_mm_sec):
        with self.lock:
            self.target_vx = vx_mm_sec
            self.target_vz = vz_mm_sec

    def home(self):
        with self.lock:
            self.target_vx = 0.0
            self.target_vz = 0.0
            self.priority_command = "$H"

    def move_absolute(self, x_mm, z_mm, speed_mm_sec):
        feedrate = speed_mm_sec * 60.0 
        with self.lock:
            self.target_vx = 0.0
            self.target_vz = 0.0
            self.priority_command = f"G90\nG1 X{x_mm:.3f} Z{z_mm:.3f} F{feedrate:.1f}"

    def get_current_position(self):
        """NEW: Safely retrieves the latest hardware position."""
        with self.lock:
            return self.actual_x_mm, self.actual_z_mm

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
                z = float(parts[2]) 
                
                with self.lock:
                    self.actual_x_mm = x
                    self.actual_z_mm = z
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
            elif 'error' in line or 'ALARM' in line:
                print(f"GRBL Response: {line}")
                break

    def _control_loop(self):
        was_jogging = False
        last_poll_time = time.time()
        
        while self.is_running:
            # --- NEW: Poll GRBL for status at 10Hz (every 0.1 seconds) ---
            current_time = time.time()
            if current_time - last_poll_time > 0.1:
                self.grbl.write(b'?') # The real-time query character
                last_poll_time = current_time
            
            # --- NEW: Read any pending serial data (like our status report) ---
            while self.grbl.in_waiting > 0:
                line = self.grbl.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('<'):
                    self._parse_status_line(line)
            
            # 1. Grab current target commands
            with self.lock:
                vx = self.target_vx
                vz = self.target_vz
                p_cmd = self.priority_command
                self.priority_command = None 
                
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
            elif abs(vx) > 0.1 or abs(vz) > 0.1:
                step_x = vx * self.DT
                step_z = vz * self.DT
                feedrate = max(abs(vx), abs(vz)) * 60.0
                
                cmd = f"$J=G91 X{step_x:.3f} Z{step_z:.3f} F{feedrate:.1f}\n"
                self.grbl.write(cmd.encode('utf-8'))
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
