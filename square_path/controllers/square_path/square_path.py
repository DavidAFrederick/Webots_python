# controllers/my_square_controller/SquarePathRobot.py
from controller import Robot, LED
# import math

class SquarePathRobot(Robot):
    def __init__(self):
        super().__init__()
        self.robot = Robot()

        # Get simulation step duration.
        self.timeStep = int(self.robot.getBasicTimeStep())
        # self.side_length = side_length

        # Setup the wheel motors
        self.left_motor = self.getDevice("left wheel")
        self.right_motor = self.getDevice("right wheel")
        
        # Configure motors for velocity control
        self.left_motor.setPosition(float('inf'))   # This sets the STOP position to infinity
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Configure the Range finders   !! There are 16 sensors around the robot >> Letters "so" and a number 0-15
        # front_sensor = self.robot.getDistanceSensor("so3")        # Get and enable the distance sensors.
        self.front_sensor = self.getDevice("so3")        # Get and enable the distance sensors.
        self.front_sensor.enable(self.timeStep)                        
        self.robot.step(self.timeStep)                            # run the clock to get a value on the sensor

        # Configure the LEDs. 
        # self.led0 = self.getDevice("led0")
        # self.led1 = self.getDevice("led1")
        # self.led2 = self.getDevice("led2")

        # if self.robot.getTime() % 1.0 < 0.5: # Blink every 1 second, on for 0.5s, off for 0.5s
        #     self.led2.set(1)
        # else:
        #     self.led2.set(0)

        # # Internal state for tracking progress
        self.corner = 0

    def move_robot_simulated_time_forward(self):
        # print (f" self.timeStep = {self.timeStep}")
        self.robot.step(self.timeStep)
        
    def move_forward(self, speed):
        self.left_motor.setVelocity(speed)
        self.right_motor.setVelocity(speed)

    def turn_right(self, speed):
        self.left_motor.setVelocity(speed)
        self.right_motor.setVelocity(-speed)

    #===( functions )===============================================================================

    def delay_seconds(self, pause_seconds):
        print ("Delay for %3f seconds" % (pause_seconds))
        count = int(pause_seconds/0.016)
        for counter in range(count):
            self.move_robot_simulated_time_forward()

    def drive_forward_at_speed(self, drive_speed):
        self.left_motor.setVelocity(drive_speed)                  # set the wheel speed to start moving the robot
        self.right_motor.setVelocity(drive_speed)
        print ("Moving Robot Forward at: %3.1f " % (drive_speed))

    def stop_robot(self):
        print ("Stopping the robot")
        self.left_motor.setVelocity(0)                # Stop the robot by setting wheel velocity
        self.right_motor.setVelocity(0)

    def turn_right_90(self):
        print ("Turning the robot right")
        drive_speed = 3.0
        self.left_motor.setVelocity  ( drive_speed)   # set the wheel speed in opposite directions
        self.right_motor.setVelocity (-drive_speed)
        self.delay_seconds(0.85)
        self.stop_robot()

    def getDistance_in_meters(self):  
        range_to_target =  ((1000 - self.front_sensor.getValue()) / 1000) * 5
        # print ("Range to target in meters: %3.1f" % (range_to_target) )
        return range_to_target


    def run(self, speed):
        """
        Main loop for the robot's behavior.
        """
        while self.step(self.timeStep ) != -1:

            if self.corner == 0:
                # Move forward on the first side
                self.move_forward(speed)

            elif self.corner == 1:
                # Turn right
                self.turn_right(speed)
                # After a fixed amount of time or orientation change, switch to next stage
                if self.getTime() > self.timeStep / 1000: # Wait a bit for turn to start
                    # For a 90-degree turn, timing is a simple but effective strategy
                    # A more robust solution uses encoders or an inertial unit
                    if self.getTime() > 1.5: # Arbitrary time for turning 90 degrees
                        print("Turn 1 completed. Moving forward...")
                        self.corner += 1

            
            # The remaining logic follows a similar pattern for the other sides
            # ... Add logic for corner 2, 3, and 4



# controllers/my_square_controller/my_square_controller.py
# from SquarePathRobot import SquarePathRobot

def main():
    print ("Before Main 2 ============= ")

    if (False):
        print ("===(Main - Original from LLM)=============================================")
        """
        Creates and runs an instance of the SquarePathRobot.
        """
        side_length = 2.0  # Length of each side of the square
        speed = 5.0        # Velocity of the robot
        
        robot_controller = SquarePathRobot()
        robot_controller.run(speed)
        print ("==========================================================================")

    if (True):

        print ("===(Main - Old)===========================================================")
        robot_one = SquarePathRobot()
        drive_velocity = 5.24

        print ("Driving down side A")
        robot_one.drive_forward_at_speed(drive_velocity)
        distance_to_wall = 1.3
        print (f"robot_one.getDistance_in_meters() {robot_one.getDistance_in_meters():5.1f}")
        robot_one.move_robot_simulated_time_forward()            

        while robot_one.getDistance_in_meters() > distance_to_wall:
            robot_one.move_robot_simulated_time_forward()    
            print (f"robot_one.getDistance_in_meters() {robot_one.getDistance_in_meters():5.1f} ")

        robot_one.stop_robot()

        robot_one.delay_seconds(1.0)

        robot_one.turn_right_90()
        robot_one.turn_right_90()
        robot_one.turn_right_90()
        robot_one.turn_right_90()

        print ("Driving back down side A")
        robot_one.drive_forward_at_speed(drive_velocity)
        distance_to_wall = 1.2
        while robot_one.getDistance_in_meters() > distance_to_wall:
            robot_one.move_robot_simulated_time_forward    
        robot_one.stop_robot()

        robot_one.delay_seconds(1.0)

        robot_one.turn_right_90()
        robot_one.turn_right_90()

        print ("Done")

    #     print ("==========================================================================")
    print ("Before Main 4 ============= ")

def main2():
    print ("Main2 ============= ")


print ("Before Main 0 ============= ")

if __name__ == "__main__":
    print ("Before Main 1 ============= ")
    main()
