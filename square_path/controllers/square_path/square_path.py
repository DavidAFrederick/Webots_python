from controller import Robot                        # Add the Robot functionality to python

robot = Robot()                                     # Create the Robot Object.
timestep = int(robot.getBasicTimeStep())            # Get the default timestep (timestep = 16)
 
leftWheel = robot.getMotor('left wheel')            # Get the wheel motors for the robot
rightWheel = robot.getMotor('right wheel')
leftWheel.setPosition(float('inf'))                 # Wheels can be controlled with position or velocity
rightWheel.setPosition(float('inf'))                # We will use velocity so set position to stop to 'infinity'

front_sensor = robot.getDistanceSensor("so3")        # Get and enable the distance sensors.
front_sensor.enable(timestep)
robot.step(timestep)                                # run the clock to get a value on the sensor


#===( functions )===============================================================================

def delay_seconds(pause_seconds):
    print ("Delay for %3f seconds" % (pause_seconds))
    count = int(pause_seconds/0.016)
    for counter in range(count):
        robot.step(timestep)

def drive_forward_at_speed(drive_speed):
    leftWheel.setVelocity(drive_speed)                  # set the wheel speed to start moving the robot
    rightWheel.setVelocity(drive_speed)
    print ("Moving Robot Forward at: %3.1f " % (drive_speed))

def stop_robot():
    print ("Stopping the robot")
    leftWheel.setVelocity(0)                # Stop the robot by setting wheel velocity
    rightWheel.setVelocity(0)

def turn_right_90():
    print ("Turning the robot right")
    drive_speed = 3.0
    leftWheel.setVelocity  ( drive_speed)   # set the wheel speed in opposite directions
    rightWheel.setVelocity (-drive_speed)
    delay_seconds(0.85)
    stop_robot()

def getDistance_in_meters():  
    range_to_target =  ((1000 - front_sensor.getValue()) / 1000) * 5
    # print ("Range to target in meters: %3.1f" % (range_to_target) )
    return range_to_target


#===( main )===============================================================================

drive_velocity = 5.24

print ("Driving down side A")
drive_forward_at_speed(drive_velocity)
distance_to_wall = 1.2
while getDistance_in_meters() > distance_to_wall:
    robot.step(timestep)    
stop_robot()

delay_seconds(1.0)

turn_right_90()
turn_right_90()
turn_right_90()
turn_right_90()

print ("Driving back down side A")
drive_forward_at_speed(drive_velocity)
distance_to_wall = 1.2
while getDistance_in_meters() > distance_to_wall:
    robot.step(timestep)    
stop_robot()

delay_seconds(1.0)

turn_right_90()
turn_right_90()

print ("Done")


#========================================================================================
