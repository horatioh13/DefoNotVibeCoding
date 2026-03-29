import serial
import time
import threading

class GrblController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.grbl = None
        
        # --- Shared State Variables ---
        self.target_vx = 0.0  # mm/sec
        self.target_vz = 0.0  # mm/sec
        self.priority_command = None # For Homing & Absolute Moves
        
        self.lock = threading.Lock()
        self.is_running = False
        self.thread = None
        self.DT = 0.05  # 50ms time slice

    def start(self):
        self.grbl = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.grbl.write(b'\r\n\r\n')
        time.sleep(2)
        self.grbl.reset_input_buffer()
        
        self.is_running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        print("GRBL Control Thread Started.")

    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join()
        if self.grbl and self.grbl.is_open:
            self.grbl.write(b'\x85') 
            time.sleep(0.1)
            self.grbl.close()

    # ==========================================
    #             USER COMMANDS
    # ==========================================

    def update_jog_velocity(self, vx_mm_sec, vz_mm_sec):
        """Call this continually from your joystick."""
        with self.lock:
            self.target_vx = vx_mm_sec
            self.target_vz = vz_mm_sec

    def home(self):
        """Triggers the GRBL homing cycle."""
        print("Commanding Homing Cycle...")
        with self.lock:
            self.target_vx = 0.0
            self.target_vz = 0.0
            self.priority_command = "$H"

    def move_absolute(self, x_mm, z_mm, speed_mm_sec):
        """Moves to an exact X/Z coordinate."""
        print(f"Commanding Absolute Move to X:{x_mm}, Z:{z_mm}")
        feedrate = speed_mm_sec * 60.0  # Convert to mm/min for GRBL
        
        with self.lock:
            self.target_vx = 0.0
            self.target_vz = 0.0
            # G90 = Absolute Distance Mode
            # G1 = Linear Feed Move
            self.priority_command = f"G90\nG1 X{x_mm:.3f} Z{z_mm:.3f} F{feedrate:.1f}"

    # ==========================================
    #             BACKGROUND THREAD
    # ==========================================

    def _wait_for_ok(self):
        """Blocks until GRBL finishes the current command."""
        while self.is_running:
            line = self.grbl.readline().decode('utf-8').strip()
            if line == 'ok':
                break
            elif 'error' in line or 'ALARM' in line:
                print(f"GRBL Response: {line}")
                break

    def _control_loop(self):
        was_jogging = False
        
        while self.is_running:
            # 1. Grab current state
            with self.lock:
                vx = self.target_vx
                vz = self.target_vz
                p_cmd = self.priority_command
                self.priority_command = None # Consume the priority command
                
            # 2. Handle Priority Commands (Homing / Absolute Moves) First
            if p_cmd:
                if was_jogging:
                    self.grbl.write(b'\x85') # Cancel jog immediately
                    time.sleep(0.2)
                    self.grbl.reset_input_buffer()
                    was_jogging = False
                
                # A priority command might have multiple lines (like G90 then G1)
                for line in p_cmd.split('\n'):
                    self.grbl.write((line + '\n').encode('utf-8'))
                    self._wait_for_ok() # This will block until the move/homing is 100% finished
                    
            # 3. Handle Live Jogging
            elif abs(vx) > 0.1 or abs(vz) > 0.1:
                step_x = vx * self.DT
                step_z = vz * self.DT
                feedrate = max(abs(vx), abs(vz)) * 60.0
                
                cmd = f"$J=G91 X{step_x:.3f} Z{step_z:.3f} F{feedrate:.1f}\n"
                self.grbl.write(cmd.encode('utf-8'))
                self._wait_for_ok()
                was_jogging = True
                
            # 4. Handle Stopping
            else:
                if was_jogging:
                    self.grbl.write(b'\x85') 
                    time.sleep(0.2)
                    self.grbl.reset_input_buffer()
                    was_jogging = False
                
                time.sleep(0.01)