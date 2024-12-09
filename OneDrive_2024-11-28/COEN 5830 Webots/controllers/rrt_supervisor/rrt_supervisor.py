from controller import Supervisor

class Node:
    """Class to represent a node in the RRT tree."""
    def __init__(self, position, parent=None, action=None):
        self.position = position
        self.parent = parent
        self.action = action

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
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # Initialize emitter
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    receiver.enable(timestep)
    
    robot = supervisor.getFromDef("e-puck")
    translation_field = robot.getField("translation")

    arena_bounds = [0, 1.0, 0, 1.0]  # x_min, x_max, y_min, y_max
    goal = [0.6, 0.6, 0]
    goal_radius = 0.1

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