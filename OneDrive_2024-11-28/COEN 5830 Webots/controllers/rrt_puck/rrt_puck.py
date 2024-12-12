from controller import Robot, Receiver, Emitter
import random
import struct

def monte_carlo_propagate(robot, num_propagations=5):
    """Perform multiple Monte Carlo propagations and return the best result."""
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    best_distance = float('inf')
    best_action = [0, 0]
    best_success = False

    # Assume initial position (0, 0) as the robot doesn't have getSelf()
    initial_position = [0, 0]
    arena_bounds = [-1, 1, -1, 1]  # Define arena bounds for collision detection

    for _ in range(num_propagations):
        # Randomly generate speeds for left and right motors
        left_speed = random.uniform(-5.28, 5.28)
        right_speed = random.uniform(left_speed - 1, left_speed + 1)
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)

        timestep = int(robot.getBasicTimeStep())
        start_time = robot.getTime()
        duration = 3  # Propagation duration in seconds

        collision_detected = False
        current_position = initial_position[:]

        while robot.step(timestep) != -1:
            current_time = robot.getTime()
            elapsed_time = current_time - start_time

            # Approximate position based on motor speeds and elapsed time
            delta_position = [(left_speed + right_speed) * 0.5 * elapsed_time, 0]  # Simplified
            current_position[0] += delta_position[0]

            # Check for collisions with arena bounds
            if not (arena_bounds[0] <= current_position[0] <= arena_bounds[1]):
                collision_detected = True
                break

            # Stop motors after the duration
            if elapsed_time >= duration:
                break

        # Stop the motors
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

        if collision_detected:
            continue

        # Calculate distance to a hypothetical goal (0.5, 0.5) as placeholder
        goal = [0.5, 0.5]
        distance_to_goal = ((current_position[0] - goal[0]) ** 2 + (current_position[1] - goal[1]) ** 2) ** 0.5

        # Update the best action if this propagation is closer to the goal
        if distance_to_goal < best_distance:
            best_distance = distance_to_goal
            best_action = [left_speed, right_speed]
            best_success = True

    return best_success, best_action

def main():
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
            command = struct.unpack('f', message)[0]

            if command == 1:
                success, action = monte_carlo_propagate(robot)

                # Send propagation result back to the supervisor
                data_bytes = struct.pack('fff', float(success), action[0], action[1])
                emitter.send(data_bytes)

if __name__ == "__main__":
    main()
