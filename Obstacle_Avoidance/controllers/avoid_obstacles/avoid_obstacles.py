"""Braitenberg-based obstacle-avoiding robot controller."""
# D.Frederick 
# Oct 2, 2025
#
#===============================================================================================
from controller import Robot, Motor, PositionSensor

#===============================================================================================
class ThymioRobot(Robot):

    def __init__(self):     # Initialize the robot

        # Get reference to the robot.
        self.robot = Robot()

        # Get simulation step duration.
        self.timeStep = int(self.robot.getBasicTimeStep())   # Results in 16 milliseconds

        # - - ( Constants ) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        # Constants of the Thymio II motors and distance sensors.
        self.maxMotorVelocity = 10
        self.distanceSensorCalibrationConstant = 360

        # - - ( Instantiate the Motors ) - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        # Get left and right wheel motors.
        self.leftMotor  = self.robot.getDevice("motor.left")
        self.rightMotor = self.robot.getDevice("motor.right")

        # Disable motor PID control mode.
        self.leftMotor.setPosition(float('inf'))         # Set the "Stop Position" to infinity
        self.rightMotor.setPosition(float('inf'))


        # - - ( Instantate the Range Sensors ) - - - - - - - - - - - - - - - - - - - - - - - - 

        # Get frontal distance sensors.
        self.outerLeftSensor    = self.robot.getDevice("prox.horizontal.0")
        self.centralLeftSensor  = self.robot.getDevice("prox.horizontal.1")
        self.centralSensor      = self.robot.getDevice("prox.horizontal.2")
        self.centralRightSensor = self.robot.getDevice("prox.horizontal.3")
        self.outerRightSensor   = self.robot.getDevice("prox.horizontal.4")

        # Enable distance sensors.
        self.outerLeftSensor.enable(self.timeStep)
        self.centralLeftSensor.enable(self.timeStep)
        self.centralSensor.enable(self.timeStep)
        self.centralRightSensor.enable(self.timeStep)
        self.outerRightSensor.enable(self.timeStep)

        # - - ( Instantiate the LEDs ) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        self.top_RGB_led = self.robot.getDevice('leds.top') # Access the  RGB LEDs
        self.bottom_left_RGB_led = self.robot.getDevice('leds.bottom.left') 
        self.bottom_right_RGB_led = self.robot.getDevice('leds.bottom.right') 

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - [Low Level Methods ]- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    def move_robot_time_forward(self) -> None:
        self.robot.step(self.timeStep)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def get_simulation_is_not_complete(self) -> bool:
        # (-1 is returned when Webots terminates the controller. Other values are the actual time step in millisec)
        if (self.robot.step(self.timeStep) != -1):   #Not complete
            is_not_complete  = True
        else:
            is_not_complete  = False
        return is_not_complete
    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def print_sensor_values(self) -> None:
        self.outerLeftSensorValue    = self.outerLeftSensor.getValue()      # Touching is 2500
        self.centralLeftSensorValue  = self.centralLeftSensor.getValue()  
        self.centralSensorValue      = self.centralSensor.getValue()      
        self.centralRightSensorValue = self.centralRightSensor.getValue() 
        self.outerRightSensorValue   = self.outerRightSensor.getValue()   

        print (f"Sensors {self.outerLeftSensorValue:5.2f} {self.centralLeftSensorValue:5.2f} {self.centralSensorValue:5.2f} {self.centralRightSensorValue:5.2f} {self.outerRightSensorValue:5.2f}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def set_both_wheel_speeds(self, percent_velocity : float) -> None:
        # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
        # self.left_velocity = self.initialVelocity - (self.centralRightSensorValue + self.outerRightSensorValue) 
        # self.right_velicity = self.initialVelocity - (self.centralLeftSensorValue + self.outerLeftSensorValue)  - self.centralSensorValue
        speed = percent_velocity * self.maxMotorVelocity
        self.leftMotor.setVelocity(speed)
        self.rightMotor.setVelocity(speed)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def set_wheel_speeds_separately(self, left_percent_velocity : float, right_percent_velocity : float) -> None:
        left_speed = left_percent_velocity * self.maxMotorVelocity
        right_speed = right_percent_velocity * self.maxMotorVelocity
        self.leftMotor.setVelocity(left_speed)
        self.rightMotor.setVelocity(right_speed)



    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - [ High Level Methods ]- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def move_forward_at_percent_speed_x_for_y_seconds(self, percent_speed : float, time_seconds : float) -> None:
        print(f"Moving forward at {percent_speed:4.1f} percent for {time_seconds:4.1f} seconds.  Current time {self.robot.getTime():5.1f}")
        self.set_both_wheel_speeds(percent_speed)
        start_time = self.robot.getTime()
        time_in_loop = 0
        # print (f"time_in_loop: {(time_in_loop*1000):5.2f}      time_seconds:  {time_seconds}")
        while (time_in_loop < time_seconds):
            self.move_robot_time_forward()
            # print (f"time_in_loop: {(time_in_loop*1000):5.2f}      time_seconds:  {time_seconds}")
            current_time = self.robot.getTime()
            time_in_loop = current_time - start_time
            # self.print_sensor_values()
        self.set_both_wheel_speeds(0)
        print (f"Stopping movement at: {current_time:5.1f}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def turn_robot_speed_time(self, percent_velocity : float, turn_duration : float) -> None:
        #  Since the robot does not have a compass or gyro, the turn is performed without feedback  of the heading change
        #  The turn is simply performed with motor speed and time. 
        #  This would not be good practice on a real robot since battery voltage drops causing the turn rate to vary. 
        #  These two parameters cause a good 90 degree turn to the right (0.5, 0.85) [Speed,Time]

        self.set_wheel_speeds_separately(percent_velocity, -percent_velocity)
        start_time = self.robot.getTime()
        time_in_loop = 0
        print (f"Turning Robot - time_in_loop: {(time_in_loop*1000):5.2f}      turn_duration:  {turn_duration}")
        while (time_in_loop < turn_duration):
            self.move_robot_time_forward()
            # print (f"time_in_loop: {(time_in_loop*1000):5.2f}      time_seconds:  {time_seconds}")
            current_time = self.robot.getTime()
            time_in_loop = current_time - start_time
        self.set_both_wheel_speeds(0)
        print (f"Stopping movement at: {current_time:5.1f}")

    # NOTE:  GETPOSITION Not support on this robot
    # def move_using_position_sensor(self, distance_to_travel : float, speed_to_travel : float) -> None:
    #     self.leftMotor.setPosition(float('inf'))         # Set the "Stop Position" to infinity
    #     self.rightMotor.setPosition(float('inf'))   
    #     self.set_both_wheel_speeds(speed_to_travel)
    #     self.current_position = self.leftMotor.getPosition()
    #     while (self.current_position < distance_to_travel):
    #         self.move_robot_time_forward()
    #         self.current_position = self.leftMotor.getPosition()
    #         print(f"Current Position:  {self.leftMotor.getPosition()}      Target Position:  {distance_to_travel}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def stop_and_pause_for_x_seconds(self, time_seconds):
        print(f"Stopping for {time_seconds:4.1f} seconds.  Current time {self.robot.getTime():5.1f}")
        self.set_both_wheel_speeds(0)
        start_time = self.robot.getTime()
        time_in_loop = 0
        while (time_in_loop < time_seconds):
            self.move_robot_time_forward()
            current_time = self.robot.getTime()
            time_in_loop = current_time - start_time
        self.set_both_wheel_speeds(0)
        print (f"Completed at: {current_time:5.1f}    {self.robot.getTime():5.1f}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def calculate_color_value(self, percent_red, percent_green, percent_blue) -> int:
        # Control the LEDs with Values betweem 0 - 32 for each color  
        # where the color positions are (RR,GG,BB).  
        # Calculate the color value, then move to correct bit position using the "* (0xFF0000)" calculation 
        red   =  (32 * percent_red)   * (0xFF0000)  
        green =  (32 * percent_green) * (0x00FF00)  
        blue  =  (32 * percent_blue)  * (0x0000FF)  
        return int(red + green + blue)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def set_color_of_top_LED(self, percent_red, percent_green, percent_blue) -> None:
        self.top_RGB_led.set(self.calculate_color_value(percent_red, percent_green, percent_blue))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def set_color_of_bottom_left_LED(self, percent_red, percent_green, percent_blue) -> None:
        self.bottom_left_RGB_led.set(self.calculate_color_value(percent_red, percent_green, percent_blue))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def set_color_of_bottom_right_LED(self, percent_red, percent_green, percent_blue) -> None:
        self.bottom_right_RGB_led.set(self.calculate_color_value(percent_red, percent_green, percent_blue))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def print_current_time(self):
        self.current_time = self.robot.getTime()
        print (f"Current time: {self.current_time:6.2f}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#===============================================================================================

robot_one = ThymioRobot()

robot_one.stop_and_pause_for_x_seconds(3)

while (robot_one.get_simulation_is_not_complete() ):    
    robot_one.print_current_time()
    robot_one.set_color_of_top_LED(1.0, 0, 0)
    robot_one.stop_and_pause_for_x_seconds(0.5)
    robot_one.turn_robot_speed_time(0.5, 1.7)    # Turn to the Right

    robot_one.set_color_of_top_LED(0, 1.0, 0)
    robot_one.stop_and_pause_for_x_seconds(0.5)
    robot_one.turn_robot_speed_time(-0.5, 1.7)   # Turn to the left


    # robot_one.turn_robot_speeds_time(0.5, 1.0)

#===============================================================================================
