import serial
import time
import sys
import tty
import termios

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

# Wake GRBL
ser.write(b"\r\n\r\n")
time.sleep(2)
ser.flushInput()

def send_gcode(cmd): 
    print("Sending:", cmd) 
    ser.write((cmd + '\n').encode()) 
    
    # Read response 
    while True: 
        response = ser.readline().decode().strip() 
        if response: 
            print("GRBL:", response) 
        if response == 'ok' or 'error' in response: 
            break

    # Ensure motors stay on
send_gcode("$1=255")

send_gcode("G21")
send_gcode("G91")  # realative positioning
# send_gcode("G90")  # absolute positioning
print("Use arrow keys to move. Ctrl+C to exit.")


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch1 + ch2 + ch3
        return ch1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

while True:
    key = get_key()
    print("Key pressed:", repr(key))

    if key == '\x1b[A':      # Up
        print("Moving up")
        send_gcode("Z1 F25")
    elif key == '\x1b[B':    # Down
        print("Moving down")
        send_gcode("Z-1 F25")
    elif key == '\x1b[C':    # Right
        print("Moving right")
        send_gcode("X1 F25")
    elif key == '\x1b[D':    # Left
        print("Moving left")
        send_gcode("X-1 F25")
