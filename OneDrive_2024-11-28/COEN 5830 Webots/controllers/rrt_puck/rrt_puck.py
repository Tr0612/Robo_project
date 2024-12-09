from controller import Robot

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize receiver
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    while robot.step(timestep) != -1:
        variable1 = 42       # Integer
        variable2 = 3.14159  # Float
        variable3 = "Sample" # String

    # Combine the data into a single string
        message = f"{variable1},{variable2},{variable3}"
        # Check if a message is received
        if receiver.getQueueLength() > 0:
            # Retrieve the message as a UTF-8 string
            received_message = receiver.getString()
            receiver.nextPacket()  # Clear the packet from the queue

            # Split the string into components
            try:
                variable1, variable2, variable3 = received_message.split(",")
                variable1 = int(variable1)
                variable2 = float(variable2)
                variable3 = str(variable3)

                # Print the received data
                print(f"Received Variables: {variable1}, {variable2}, {variable3}")
            except ValueError as e:
                print(f"Error parsing message: {e}")
                continue  # Skip to the next iteration if parsing fails
            
if __name__ == "__main__":
    main()
