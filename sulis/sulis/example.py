import time

gantry = GrblController(port='/dev/ttyUSB0')
gantry.start()

try:
    # 1. Home the machine at startup
    gantry.home()
    
    # We add a sleep here in the main script just so we don't spam commands,
    # but the background thread will actually wait for homing to finish on its own.
    time.sleep(1) 

    # 2. Move to a starting position (e.g. center of the 80cm track, 5cm down)
    # Speed is 20 mm/sec
    gantry.move_absolute(x_mm=400.0, z_mm=-50.0, speed_mm_sec=20.0)
    time.sleep(1)

    # 3. Enter your joystick loop
    print("Entering manual control...")
    while True:
        # If joystick pushed:
        gantry.update_jog_velocity(vx_mm_sec=15.0, vz_mm_sec=0.0)
        time.sleep(1)
        
        # If joystick released:
        gantry.update_jog_velocity(vx_mm_sec=0.0, vz_mm_sec=0.0)
        time.sleep(1)
        
        # You could even map a button on the controller to an absolute move!
        # if button_y_pressed:
        #     gantry.move_absolute(x_mm=0.0, z_mm=0.0, speed_mm_sec=50.0)

except KeyboardInterrupt:
    pass
finally:
    gantry.stop()