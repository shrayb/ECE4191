from PinAssignment import *
import RPi.GPIO as GPIO

from threading import Thread
from multiprocessing import Process, Value, Queue

from BaseClasses import *
from Robot import Robot

# Rasp pi stuff
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Create component classes
left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a)

front_left_sonic = Ultrasonic(echo_pin=front_left_sonic_echo, trig_pin=front_left_sonic_trig, x_offset=0.0875, y_offset=0.060, theta=22.5*math.pi/180, reading_index=0, maximum_read_distance=0.15)
front_right_sonic = Ultrasonic(echo_pin=front_right_sonic_echo, trig_pin=front_right_sonic_trig, x_offset=0.0875, y_offset=-0.060, theta=-22.5*math.pi/180, reading_index=1, maximum_read_distance=0.15)
middle_sonic = Ultrasonic(echo_pin=middle_sonic_echo, trig_pin=middle_sonic_trig, x_offset=0.100, y_offset=0, theta=0, reading_index=2, maximum_read_distance=0.15)
left_sonic = Ultrasonic(echo_pin=left_sonic_echo, trig_pin=left_sonic_trig, x_offset=-0.047, y_offset=0.123, theta=math.pi/2, reading_index=3, maximum_read_distance=0.15)
right_sonic = Ultrasonic(echo_pin=right_sonic_echo, trig_pin=right_sonic_trig, x_offset=-0.047, y_offset=-0.123, theta=-math.pi/2, reading_index=4, maximum_read_distance=0.15)

limit_switch = LimitSwitch(distance=0.1, switch_pin=limit_switch_pin)
package_sonic = Ultrasonic(echo_pin=package_sonic_echo, trig_pin=package_sonic_trig, x_offset=0, y_offset=0, theta=0, reading_index=-1, maximum_read_distance=0.3)
# colour_sensor = ColourSensor(s3=s3, s2=s2, signal=colour_sensor_signal)

# Create robot class and instantiate component classes
pose = Pose(0.3, 0.4, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor
robot.front_left_ultrasonic = front_left_sonic
robot.front_right_ultrasonic = front_right_sonic
robot.middle_ultrasonic = middle_sonic
robot.right_ultrasonic = right_sonic
robot.left_ultrasonic = left_sonic
robot.limit_switch = limit_switch
robot.package_ultrasonic = package_sonic
# robot.colour_sensor = colour_sensor

# Start encoder process
with Manager() as manager:
    left_motor = robot.left_motor
    right_motor = robot.right_motor

    encoder_process = Process(target=robot.encoder_process, args=(left_motor, right_motor))
    encoder_process.start()

# Initial thread start for localisation
ultrasonic_thread = Thread(target=robot.ultrasonic_thread)
ultrasonic_thread.start()

drive_thread = Thread(target=robot.drive_thread)
drive_thread.start()

def mainloop():
    try:
        while True:
            # Check colour sensor for package
            if robot.current_goal is None and not robot.delivering:
                # # Re-localise the robot with a corner
                # robot.do_localise = True
                #
                # # Wait until the robot has finished localising
                # print("Robot re-localising...")
                # while robot.do_localise:
                #     sleep(0.1)
                # print("Robot localised at: (", robot.pose.x, robot.pose.y, ")")

                # End all the threads to prepare for scanning
                robot.end_all_threads = True

                # Scan for a new package
                while robot.package is None:
                    print("Scanning for new package...")
                    package_id = robot.scan_package_ultrasonic()

                    if package_id != 3:
                        robot.package = Package(package_id)
                        break

                print("Scanned package:", package_id)

                # Make the current goal the package delivery position and tell the robot its now delivering
                robot.current_goal = robot.package.destination_pose
                robot.delivering = True
                robot.end_all_threads = False

                # Start threads
                ultrasonic_thread = Thread(target=robot.ultrasonic_thread)
                ultrasonic_thread.start()

                drive_thread = Thread(target=robot.drive_thread)
                drive_thread.start()

            # If the robot is at the deposit zone and ready to deposit
            if robot.current_goal is None and robot.delivering:
                robot.deposit_package()

            sleep(0.001)

    # Handle Control-C to stop motors
    except KeyboardInterrupt:
        robot.left_motor.speed = 0
        robot.right_motor.speed = 0
        robot.left_motor.stop()
        robot.right_motor.stop()


if __name__ == "__main__":
    mainloop()

