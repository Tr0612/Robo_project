# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor, Emitter
import sys
import struct
import numpy as np
import time

def main():
    supervisor = Supervisor()
    puck = supervisor.getFromDef("e-puck")
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    emitter.setChannel(10)
    receiver.setChannel(10)
    timestep = int(supervisor.getBasicTimeStep())
    receiver.enable(timestep)
    while supervisor.step(timestep) != -1:
        puck.getField("translation").setSFVec3f([0.3,0,0])
        message = struct.pack('f', 1)
        emitter.send(message)
        current_state = puck.getField("translation").getSFVec3f()
        while(receiver.getQueueLength() == 0):
            supervisor.step(timestep)
        data = receiver.getBytes()
        success, left_speed, right_speed = struct.unpack('fff', data)
        receiver.nextPacket()
        time.sleep(500)
if __name__ == "__main__":
    main()
