# from controller import Robot, Emitter, Receiver, Motor, DistanceSensor,Supervisor
# import numpy as np
# import struct
# import time

# # Monte Carlo propagation function
# def monte_carlo_propagate(robot, left_motor, right_motor, proximity_sensors, duration):
#     """
#     Perform Monte Carlo propagation by assigning random velocities to the motors
#     and checking for collisions using proximity sensors.
#     """
#     success = True  # Assume success until a collision is detected
#     timestep = int(robot.getBasicTimeStep())
#     start_time = robot.getTime()
#     max_duration = duration  # Random duration for this propagation
    
#     # Enable proximity sensors
#     for sensor in proximity_sensors:
#         sensor.enable(timestep)
    
#     # Assign random velocities
#     left_velocity = np.random.uniform(-6.28, 6.28)  # Random velocity for left motor
#     right_velocity = np.random.uniform(-6.28, 6.28)  # Random velocity for right motor
    
#     # Set motor velocities
#     left_motor.setVelocity(left_velocity)
#     right_motor.setVelocity(right_velocity)
    
#     # Propagate robot motion for the random duration
#     while robot.step(timestep) != -1:
#         # Check if duration is reached
#         if robot.getTime() - start_time >= max_duration:
#             break
        
#         # Collision checking using proximity sensors
#         for sensor in proximity_sensors:
#             if sensor.getValue() > 80:  # Threshold for detecting collision
#                 success = False
#                 break
        
#         if not success:
#             break
    
#     # Stop the motors
#     left_motor.setVelocity(0)
#     right_motor.setVelocity(0)
    
#     # Return success and velocities
#     return success, left_velocity, right_velocity

# # Main function
# def main():
#     robot = Robot()
#     timestep = int(robot.getBasicTimeStep())
    
#     # Initialize devices
#     left_motor = robot.getDevice("left wheel motor")
#     right_motor = robot.getDevice("right wheel motor")
#     emitter = robot.getDevice("emitter")
#     receiver = robot.getDevice("receiver")
#     # proximity_sensors = [robot.getDevice(f"ps{i}") for i in range(8)]
    
#     # Motor configuration
#     left_motor.setPosition(float('inf'))
#     right_motor.setPosition(float('inf'))
#     left_motor.setVelocity(0)
#     right_motor.setVelocity(0)
    
#     # Enable receiver
#     receiver.enable(timestep)
    
#     # Enable proximity sensors
#     # for sensor in proximity_sensors:
#         # sensor.enable(timestep)
    
#     while robot.step(timestep) != -1:
#         # Check for received messages
#         if receiver.getQueueLength() > 0:
#             print("loop check")
#             packed_message = receiver.getBytes()
#             receiver.nextPacket()
            
#             # Decode the message
#             message = struct.unpack('f',packed_message)[0]
#             print(f"message {message}")
#             if command == 1:
#                 # print(f"Received Command: {command}, ID: {command_id}, Duration: {duration}")
                
#                 # Perform Monte Carlo propagation
#                 duration = np.random.uniform(0.5, 1.5)  # Random propagation duration
#                 success, left_velocity, right_velocity = monte_carlo_propagate(
#                     robot, left_motor, right_motor, proximity_sensors, duration
#                 )
#                 to_supervisor_msg = f"{success},{left_velocity},{right_velocity}"
#                 print(to_supervisor_msg)
#                 # Send results back to supervisor
#                 # packed_data = struct.pack("fff", float(success), left_velocity, right_velocity)
#                 emitter.send(to_supervisor_msg.encode('utf-8'))


# if __name__ == "__main__":
#     main()

from controller import Supervisor

def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    robot = supervisor.getFromDef("e-puck")
    # Initialize emitter
    emitter = supervisor.getDevice("emitter")

    # Example variables to send
    variable1 = 42       # Integer
    variable2 = 3.14159  # Float
    variable3 = "Sample" # String

    # Combine the data into a single string
    message = f"{variable1},{variable2},{variable3}"

    while supervisor.step(timestep) != -1:
        # Send the string message
        emitter.send(message.encode('utf-8'))
        print(f"Sent Message: {message}")
        break  # Send only once in this example

if __name__ == "__main__":
    main()