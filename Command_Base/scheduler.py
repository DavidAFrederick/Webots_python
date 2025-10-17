from squarepathrobot import SquarePathRobot


robot_one = SquarePathRobot()


while (robot_one.get_simulation_is_not_complete() ):    
    robot_one.print_current_time()
    robot_one.move_using_position_sensor()