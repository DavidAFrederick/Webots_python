from command import DriveCommand

from controller import Robot


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

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def get_simulation_is_not_complete(self) -> bool:
        # (-1 is returned when Webots terminates the controller. Other values are the actual time step in millisec)
        if (self.robot.step(self.timeStep) != -1):   #Not complete
            is_not_complete  = True
        else:
            is_not_complete  = False
        return is_not_complete
    
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def print_current_time(self):
        current_time = self.robot.getTime()
        print (f"Current time: {current_time}")

    
#--------------------------------------------------------------------------------------------

def main():
    robot_one = SquarePathRobot()
    drivecommandset = DriveCommand()
    drivecommandset.initialize()

    while (robot_one.get_simulation_is_not_complete() ):   
        robot_one.move_robot_simulated_time_forward()

        
        drivecommandset.execute()
        if (drivecommandset.isFinished()):
            drivecommandset.end()


        robot_one.print_current_time()


#--------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------

if __name__ == "__main__":
    main()
#--------------------------------------------------------------------------------------------
