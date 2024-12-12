"""rrt_puck controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import struct
import random

# create the Robot instance.

def monte_carlo_propagate():
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    left_speed = random.uniform(-5.28, 5.28)
    right_speed = random.uniform(left_speed-1, left_speed+1)
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
    timestep = 32
    start_time = robot.getTime()
    duration = 3
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        elapsed_time = current_time - start_time

        if elapsed_time < duration:
            pass
        else:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            return True, [left_speed, right_speed]



robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
emitter.setChannel(10)
receiver.setChannel(10)
receiver.enable(timestep)

while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        receiver.nextPacket()
        message = struct.unpack('f', message)[0]
        if message == 1:
            success, action = monte_carlo_propagate()
            print(success, type(success), action, type(action[0]))
            data_bytes = struct.pack('fff', float(success), action[0], action[1])
            emitter.send(data_bytes)

# Enter here exit cleanup code.
