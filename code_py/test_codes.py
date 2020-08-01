# import libraries here
import gpiozero
import serial
import cv2
import motor
import vision
import time

# open serial port
ser = serial.Serial(
                    port='/dev/ttyUSB0',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=3
                    )
index_image = 0


def main():
    wait = True
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Hello' not in message_from_serial.decode("utf-8")
    print(message_from_serial.decode("utf-8"))
    print("KUKA WANTS")

    # Preset several general scan locations (where the face images will be captured by the camera)
    scan_locations = []
    scan_locations.append([1400.0, 300.0, 800.0, 30.0, 90.0, 0.0])
    scan_locations.append([1500.0, -450.0, 1200.0, 30.0, 90.0, 0.0])
    scan_locations.append([1500.0, -500.0, 1400.0, 30.0, 90.0, 0.0])
    scan_locations.append([1500.0, -550.0, 1400.0, 30.0, 90.0, 0.0])
    scan_locations.append([1500.0, -550.0, 1400.0, 30.0, 90.0, 0.0])

    # Preset refill location and the distance to lower the syringe for refilling
    refill_location = [1640., 785., 380., 30., 90., 0.]
    submerge_distance = 130.


    for i in range(0, len(scan_locations)):
        paint_locations = scan(scan_locations[i])   # Scan the environment in the provided TCP, return a list of paint locations
        for j in range(0, len(paint_locations)):
            paint(paint_locations[j], scan_locations[i])

    # go to refill after paint all the detected images
    refill(refill_location, submerge_distance)
    home()  # Let Kuka to go to home position
    print("Going home sent")

    time.sleep(2)

    ser.close()  # Close serial port
    exit()

def refill(location, submerge_distance):

    loc_x = location[0]  # Target Robot Location X
    loc_y = location[1]  # Target Robot Location Y
    loc_z = location[2]  # Target Robot Location Z
    loc_a = location[3]  # Target Robot Location A
    loc_b = location[4]  # Target Robot Location B
    loc_c = location[5]  # Target Robot Location C

    """
    Send Serial to robot to move to location over refill tank
        # Send "int               float   float   float   float   float   float"
        # Send "COMMAND(1=move)   POS_X   POS_Y   POS_Z   POS_A   POS_B   POS_C"
    Get Serial from robot when it moved to the location
        # Receive "KUKA Moving"
        # Receive "Finish"
    
    Send Serial to robot to move to location in refill tank
        # Send "int               float   float   float   float   float   float"
        # Send "COMMAND(1=move)   POS_X   POS_Y   POS_Z   POS_A   POS_B   POS_C"
    Get Serial from robot when it moved to the location
        # Receive "KUKA Moving"
        # Receive "Finish"
    
    Gear move to refill
    
    Send Serial to robot to move to location over refill tank
        # Send "int             float   float   float   float   float   float"
        # Send "COMMAND(1=move) POS_X   POS_Y   POS_Z   POS_A   POS_B   POS_C"
    Get Serial from robot when it moved to the location
        # Receive "KUKA Moving"
        # Receive "Finish"
    """

    if (True):
        # move syringe over water tank
        time.sleep(2)  # To avoid problems caused by transmitting delay
        message0 = "%i %f %f %f %f %f %f" % (1, loc_x, loc_y, loc_z, loc_a, loc_b, loc_c)
        ser.write(message0)
        wait = True
        while(wait):
            message_from_serial = ser.readline()
            wait = 'Finish' not in message_from_serial
        time.sleep(2)  # To avoid problems caused by transmitting delay
        
        # move robot to submerge syringe in water tank
        message0 = "%i %f %f %f %f %f %f" % (1, loc_x, loc_y, loc_z - submerge_distance, loc_a, loc_b, loc_c)
        ser.write(message0)
        wait = True
        while (wait):
            message_from_serial = ser.readline()
            wait = 'Finish' not in message_from_serial

        # move_gear to refill
        print("start refilling")
        stepper = motor.Motor([6, 13, 19, 26])
        time.sleep(5)
        stepper.step(4000, True)
        print("refilled")
        time.sleep(2)  # To avoid problems caused by transmitting delay
        
        # write a string with move command
        message0 = "%i %f %f %f %f %f %f" % (1, loc_x, loc_y, loc_z, loc_a, loc_b, loc_c)
        ser.write(message0)
        wait = True
        while (wait):
            message_from_serial = ser.readline()
            wait = 'Finish' not in message_from_serial
    return


def home():

    """
    Send Serial to robot to move to home
        # Send "int             float       float       float       float       float       float(0)"
        # Send "COMMAND(2=home) POS_X(0)    POS_Y(0)    POS_Z(0)    POS_A(0)    POS_B(0)    POS_C(0)"
    Get Serial from robot when it moved to the location
        # Receive "KUKA Returning Home"
        # Receive "Finish"
    Send Serial to end program
        # Send "int             float       float       float       float       float       float(0)"
        # Send "COMMAND(2=home) POS_X(0)    POS_Y(0)    POS_Z(0)    POS_A(0)    POS_B(0)    POS_C(0)"
    Get Serial from robot when it disconnects
        # Receive "KUKA Terminate Connection"
    """
    wait = True
    ser.write("%i %f %f %f %f %f %f".format(int(2), float(0), float(0), float(0), float(0), float(0), float(0)))
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Finish' not in message_from_serial

    ser.write("%i %f %f %f %f %f %f".format(int(3), float(0), float(0), float(0), float(0), float(0), float(0)))
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Terminate' not in message_from_serial

    return


def paint(location, scan_location):

    loc_x = scan_location[0] - location[0] - 50 #location[0]  # Target Robot Location X
    loc_y = scan_location[1] + location[1] + 70 #location[1]  # Target Robot Location Y
    loc_z = scan_location[2] - location[2] + 300 #location[2]  # Target Robot Location Z
    # A connecting piece, to which our end effector attached, was mounted 30 degree clockwise rotated to the robot
    loc_a = 30.  # Target Robot Location A,
    # We want to paint with the robot only facing vertically down
    loc_b = 90.  # Target Robot Location B
    loc_c = 0.  # Target Robot Location C

    """
    Send Serial to robot to move to location
        # Send "int             float   float   float   float   float   float"
        # Send "COMMAND(1=move) POS_X   POS_Y   POS_Z   POS_A   POS_B   POS_C"
    Get Serial from robot when it moved to the location
        # Receive "KUKA Moving"
        # Receive "Finish"
    Apply Paint
    Check to refill
    """

    # move syringe at paint location
    message0 = "%i %f %f %f %f %f %f" % (int(1), loc_x, loc_y, loc_z, loc_a, loc_b, loc_c)
    ser.write(message0)
    wait = True
    while (wait):
        message_from_serial = ser.readline()
        wait = 'FinishPaint' not in message_from_serial
    time.sleep(2)  # To avoid problems caused by transmitting delay

    # Move gear to apply paint
    motor.GPIO_init()
    print("gear start moving")
    stepper = motor.Motor([6, 13, 19, 26])
    time.sleep(5)  # To avoid problems caused by transmitting delay
    stepper.step(1000, False)
    print("gear moved")
    return


def scan(location):

    global index_image

    loc_x = location[0]  # Target Robot Location X
    loc_y = location[1]  # Target Robot Location Y
    loc_z = location[2]  # Target Robot Location Z
    loc_a = location[3]  # Target Robot Location A
    loc_b = location[4]  # Target Robot Location B
    loc_c = location[5]  # Target Robot Location C

    """
    Send Serial to robot to move to location
        # Send "int             float   float   float   float   float   float   float"
        # Send "COMMAND(1=move) POS_X   POS_Y   POS_Z   POS_A   POS_B   POS_C   COMMENT(0)"
    Get Serial from robot when it moved to the location
        # Receive "KUKA Moving"
        # Receive "Finish"
    Camera Read
    Return a list of paint locations
    """

    # move to scan locations
    print("Pi sending locations")
    message0 = "%i %f %f %f %f %f %f" % (1, loc_x, loc_y, loc_z, loc_a, loc_b, loc_c)
    ser.write(message0)
    print("location sent")
    wait = True
    while (wait):
        message_from_serial = ser.readline()
        wait = 'FinishScan' not in message_from_serial
    time.sleep(2)  # To avoid problems caused by transmitting delay

    # Open Camera
    camera = vision.vision_init()
    result = vision.detect_position_face_multiple_times(camera, 10, index_image)
    print(result)
    index_image += 1
    # Return paint locations
    time.sleep(3)  # To avoid problems caused by transmitting delay
    return result

main()
