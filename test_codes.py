# import libraries here
import gpiozero
import serial
# from math import pi
import motor
import vision

# open serial port
ser = serial.Serial('/dev/ttyUSB0', baudrate=9600)  # communicate with arduino '/dev/ttyACM0'

def main():
    scan_locations = [0., 0., 0., 0., 0., 180]
    refill_location = [0., 0., 0., 0., 0., 180]
    submerge_distance = 20.



    volumn = 20
    for i in range(0,len(scan_locations)):
        paint_locations = scan(scan_locations[i])   # Scan the environment in the provided TCP, return a list of paint locations
        for j in range(0,len(paint_locations)):
            paint(paint_locations[j])
            volumn -= 1
            if volumn < 5:
                refill(refill_location, submerge_distance)
            volumn += 15
    home() # Let Kuka to go to home position
    ser.close()  # Close serial port
    exit()


main()


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

    # volume = 10  # Set volume to the volume of fluid in the syringe based on the motor location

    if (True):
        # move syringe over water tank
        wait = True
        ser.write("%i %f %f %f %f %f %f".format(int(1), float(loc_x), float(loc_y), float(loc_z), float(loc_a), float(loc_b), float(loc_c)))
        while(wait):
            message_from_serial = ser.readline()
            wait = 'Finish' not in message_from_serial

        # move robot to submerge syringe in water tank
        wait = True
        ser.write("%i %f %f %f %f %f %f".format(int(1), float(loc_x), float(loc_y), float(loc_z - submerge_distance), float(loc_a), float(loc_b), float(loc_c)))
        while (wait):
            message_from_serial = ser.readline()
            wait = 'Finish' not in message_from_serial

        # move_gear to refill


        # write a string with move command
        wait = True
        ser.write("%i %f %f %f %f %f %f %f".format(int(1), float(loc_x), float(loc_y), float(loc_z), float(loc_a), float(loc_b), float(loc_c)))
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

    ser.write("%i %f %f %f %f %f %f".format(int(2), float(0), float(0), float(0), float(0), float(0), float(0)))
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Finish' not in message_from_serial

    ser.write("%i %f %f %f %f %f %f".format(int(3), float(0), float(0), float(0), float(0), float(0), float(0)))
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Terminate' not in message_from_serial

    return


def paint(location):

    loc_x = location[0]  # Target Robot Location X
    loc_y = location[1]  # Target Robot Location Y
    loc_z = location[2]  # Target Robot Location Z
    loc_a = location[3]  # Target Robot Location A
    loc_b = location[4]  # Target Robot Location B
    loc_c = location[5]  # Target Robot Location C

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
    wait = True
    ser.write("%i %f %f %f %f %f %f".format(int(1), float(loc_x), float(loc_y), float(loc_z), float(loc_a), float(loc_b), float(loc_c)))
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Finish' not in message_from_serial

    # Move gear to apply paint
    motor.GPIO_init()
    stepper = motor.Motor([6, 13, 19, 26])
    stepper.step(1000, False)

    return


def scan(location):

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
    wait = True
    ser.write("%i %f %f %f %f %f %f".format(int(1), float(loc_x), float(loc_y), float(loc_z), float(loc_a), float(loc_b), float(loc_c)))
    while (wait):
        message_from_serial = ser.readline()
        wait = 'Finish' not in message_from_serial

    # Open Camera
    camera = vision.vision_init()
    locations = vision.detect_position_green(camera)
    # Return paint locations

    return locations
