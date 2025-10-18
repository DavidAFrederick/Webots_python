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
        self.set_RGB_LED_1()
        self.set_both_wheel_speeds(percent_speed)
        start_time = self.robot.getTime()
        time_in_loop = 0
        # print (f"time_in_loop: {(time_in_loop*1000):5.2f}      time_seconds:  {time_seconds}")
        while (time_in_loop < time_seconds):
            self.move_robot_time_forward()
            # print (f"time_in_loop: {(time_in_loop*1000):5.2f}      time_seconds:  {time_seconds}")
            current_time = self.robot.getTime()
            time_in_loop = current_time - start_time
            self.print_sensor_values()
        self.set_both_wheel_speeds(0)
        print (f"Stopping movement at: {current_time:5.1f}")

    def move_using_position_sensor(self, distance_to_travel : float, speed_to_travel : float) -> None:
        self.leftMotor.setPosition(float('inf'))         # Set the "Stop Position" to infinity
        self.rightMotor.setPosition(float('inf'))   
        self.set_both_wheel_speeds(speed_to_travel)
        self.current_position = self.leftMotor.getPosition()
        while (self.current_position < distance_to_travel):
            self.move_robot_time_forward()
            self.current_position = self.leftMotor.getPosition()
            print(f"Current Position:  {self.leftMotor.getPosition()}      Target Position:  {distance_to_travel}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def stop_and_pause_for_x_seconds(self, time_seconds):
        print(f"Stopping for {time_seconds:4.1f} seconds.  Current time {self.robot.getTime():5.1f}")
        self.set_RGB_LED_2()
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
    def set_RGB_LED_1(self):    # (0xFF0000)
        self.top_RGB_led.set(0xFF0000)
        self.bottom_left_RGB_led.set(0x0000FF)
        self.bottom_right_RGB_led.set(0x000000)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def set_RGB_LED_2(self):    # (0xFF0000)
        self.top_RGB_led.set(0x00FF00)
        self.bottom_left_RGB_led.set(0x000000)
        self.bottom_right_RGB_led.set(0x0000FF)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def print_current_time(self):
        self.current_time = self.robot.getTime()
        print (f"Current time: {self.current_time:6.2f}")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


#===============================================================================================

robot_one = ThymioRobot()

# for counter in range (100):
#     print(f"Loop Number {counter}")
#     robot_one.move_forward_at_percent_speed_x_for_y_seconds(0.5, 2)
#     robot_one.stop_and_pause_for_x_seconds(1)


while (robot_one.get_simulation_is_not_complete() ):    
    # robot_one.print_current_time()
    robot_one.move_using_position_sensor()

#
#
#
#

    
    # Read values from four distance sensors and calibrate.

    # leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
    # rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue)
    # print(f">> centralRightSensorVa÷lue :{centralRightSensorValue:5.2f}    left_velocity: {left_velocity:5.1f}   right_velecity: {right_velicity:5.1f}")


    #===============================================================================================
