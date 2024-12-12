from controller import Robot,Supervisor

# robot = Robot()
TIME_STEP = 16
supervisor = Supervisor()

# Access motors
joint1 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint1')
joint2 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint2')
joint3 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint3')
joint4 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint4')
joint5 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint5')
joint6 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint6')
joint7 = supervisor.getFromDef("panda").getFromProtoDef('panda_joint7')

def move_joints(positions, duration):
    joint1.setJointPosition(positions[0])
    joint2.setJointPosition(positions[1])
    joint3.setJointPosition(positions[2])
    joint4.setJointPosition(positions[3])
    joint5.setJointPosition(positions[4])
    joint6.setJointPosition(positions[5])
    joint7.setJointPosition(positions[6])
    for _ in range(int(duration / TIME_STEP)):
        supervisor.step(TIME_STEP)

# Initial position
initial_position = [0, 0, 0, -1.56, -2.0, 3.0, 1.0]  # Modify as needed
move_joints(initial_position, 1000)


# initial_position = 
# move_joints([0, 0, 0, -1.56, -2.0, 3.0, 1.0] , 1000)
# move_joints([0, 0, 0, -2, -2.97, 3.0, 1.0] , 1000)


# move_joints([0, 0, 0,-1.5,-2, 3,1], 1000)

# move_joints([0, 0, 0,0,-2, 3.82], 1000)
# Path for the letter "C"
# Start at the top of the "C"
# move_joints([0, -0.5, 0, -1.2, 0.5, 1.8], 1000)
# move_joints([0, -0.2, 0, -2.2, -2.5, 3.5], 1000)

# Move towards the top curve of "C"
# move_joints([0, -0.5, 0, -2.0, -2.0, 3.2], 1000)

# Move along the top curve to the right
# move_joints([0, -0.8, 0, -1.8, -1.5, 3.0], 1000)

# Move downwards along the right curve of "C"
# move_joints([0, -1.0, 0, -2.0, -1.2, 2.8], 1000)

# Return to the starting position
# move_joints([0, 0, 0, -2.5, -2.97, 3.82], 1000)
# Move to the middle left
# move_joints([0, -0.8, 0, -1.0, 0.3, 1.5], 1000)

# Move to the bottom left of "C"
# move_joints([0, -1.0, 0, -0.8, 0.0, 1.2], 1000)

# Finish the "C" by moving to the bottom right
# move_joints([0, -1.2, 0, -1.0, -0.3, 1.0], 1000)

# Return to initial position
# move_joints([0, 0, 0, -1.5, 0, 1.5], 1000)
