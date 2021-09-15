from interbotix_xs_modules.arm import InterbotixManipulatorXS

# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'
# Let the user select the position
while mode != 'q':
    mode=input("[h]ome, [s]leep, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose()
        joint_positions = [0, 0, 0, 1.57]
        robot.arm.go_to_home_pose()
        robot.arm.set_joint_positions(joint_positions)
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()