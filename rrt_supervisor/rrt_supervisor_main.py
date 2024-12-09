from controller import Supervisor
import numpy as np
import struct

class Node:
    """Class to represent a node in the RRT tree."""
    def __init__(self, position, parent=None, action=None):
        self.position = position
        self.parent = parent
        self.action = action  # (left_velocity, right_velocity)

def randomly_sample_state(goal, arena_bounds, goal_bias=0.5):
    """Randomly sample a state with 50% goal bias."""
    # num = np.random.random()
    num =0.7
    # print(f"random number {num}")
    if  num < goal_bias:
        return np.array(goal)
    else:
        return np.array([
            np.random.uniform(arena_bounds[0], arena_bounds[1]),
            np.random.uniform(arena_bounds[2], arena_bounds[3]),
            0  # Assume z is fixed
        ])

def main():
    # Initialize Webots Supervisor
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    
    # Devices
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    receiver.enable(timestep)
    
    # E-puck robot
    robot = supervisor.getFromDef("e-puck")
    translation_field = robot.getField("translation")
    
    # Arena and goal parameters
    arena_bounds = [0, 1.0, 0, 1.0]  # x_min, x_max, y_min, y_max
    goal = [0.6, 0.6, 0]
    goal_radius = 0.1
    
    # RRT tree initialization
    initial_position = np.array(translation_field.getSFVec3f())
    rrt_tree = [Node(position=initial_position)]
    
    # Simulation loop
    while supervisor.step(timestep) != -1:
        # Randomly sample a state
        sampled_state = randomly_sample_state(goal, arena_bounds)
        print(f"Sampled state: {sampled_state}")
        nearest_node = Node(position=sampled_state)
        # Find the nearest node in the tree
        # nearest_node = min(rrt_tree, key=lambda node: np.linalg.norm(node.position - sampled_state))
        print(f"Nearest node: {nearest_node.position}")
        
        # Teleport the robot to the nearest node
        translation_field.setSFVec3f(nearest_node.position.tolist())
        initial_position = np.array(translation_field.getSFVec3f())
        print(f"moved {initial_position}")
        supervisor.step(timestep)
        
        # Request Monte Carlo propagations
        # command = "MONTE_CARLO_PROPAGATION"
        # command_id = 1  # Example ID
        # duration = 1.0  # Example duration (seconds)

    # Pack the message with struct
        packed_message = struct.pack('f',1)
        emitter.send(packed_message)
        print(f"Sent packed message: {packed_message}")
        successful_propagations = []
        while len(successful_propagations) < 5:
            if receiver.getQueueLength() > 0:
                data = receiver.getData()
                receiver.nextPacket()
                
                # Unpack the received data
                success, left_velocity, right_velocity = struct.unpack("f??", data[:9])
                
                if success:
                    # Get the robot's current position after propagation
                    final_position = np.array(translation_field.getSFVec3f())
                    successful_propagations.append((final_position, (left_velocity, right_velocity)))
                    
                # Teleport the robot back to the nearest node
                translation_field.setSFVec3f(nearest_node.position.tolist())
                supervisor.step(timestep)
        
        # Choose the best propagation (closest to the sampled state)
        if successful_propagations:
            best_propagation = min(successful_propagations, key=lambda x: np.linalg.norm(x[0] - sampled_state))
            final_position, action = best_propagation
            
            # Add the new node to the tree
            new_node = Node(position=final_position, parent=nearest_node, action=action)
            rrt_tree.append(new_node)
            print(f"Added new node at position: {new_node.position}")
            
            # Check if the goal is reached
            if np.linalg.norm(new_node.position - goal) <= goal_radius:
                print("GOAL REACHED!")
                
                # Collect the final path
                path = []
                current = new_node
                while current.parent is not None:
                    path.append(current.action)
                    current = current.parent
                path.reverse()
                
                # Transmit the path for replay
                emitter.send("GOAL_REACHED".encode('utf-8'))
                for action in path:
                    left_velocity, right_velocity = action
                    data = struct.pack("ff", left_velocity, right_velocity)
                    emitter.send(data)
                    supervisor.step(int(1000 * 0.5))  # Execute for 0.5 seconds
                
                break


if __name__ == "__main__":
    main()




 packed_message = struct.pack('f',5)

    while supervisor.step(timestep) != -1:
        # Send the packed message to the robot
        emitter.send(packed_message)
        # print(f"Sent message: Command={packed_message}")
        break 