# Code works with quadrirotor model
# scene_quadcopter.ttt
# simRemoteApi.start(19999)

import sim
import time
import sys


class Simulation:

    def init(self):
        print('Starting Simulation')
        sim.simxFinish(-1)  # just in case, close all opened connections

        self.clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

        return self.clientID

    def close(self):
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        # You can guarantee this with (for example):
        sim.simxGetPingTime(self.clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(self.clientID)


class Quadcopter:

    def __init__(self, clientID):
        self.clientID = clientID
        self.quadHandle = None
        self.pos = [0, 0, 0]
        self.prop = ['propeller1Vel', 'propeller2Vel', 'propeller3Vel', 'propeller4Vel']
        self.prop_vel = [0.0, 0.0, 0.0, 0.0]
        self.orig_location = [0, 0, 0]
        self.curr_location = [0, 0, 0]
        self.target_z = 0.0

        self.pParam = 2
        self.iParam = 0
        self.dParam = 0
        self.vParam = -2

        self.cumul = 0
        self.lastE = 0
        self.pAlphaE = 0
        self.pBetaE = 0
        self.psp2 = 0
        self.psp1 = 0

        self.prevEuler = 0

    def init(self):
        err, self.quadHandle = sim.simxGetObjectHandle(self.clientID, 'Quadricopter', sim.simx_opmode_blocking)
        print("Quadricopter handle return code")
        print(err)

        self.init_imu()
        self.init_prop()


    def init_imu(self):
        ret = [0, 0, 0, 0, 0, 0]
        ret[0], _ = sim.simxGetFloatSignal(self.clientID, 'gyroX', sim.simx_opmode_streaming)
        ret[1], _ = sim.simxGetFloatSignal(self.clientID, 'gyroY', sim.simx_opmode_streaming)
        ret[2], _ = sim.simxGetFloatSignal(self.clientID, 'gyroZ', sim.simx_opmode_streaming)

        ret[3], _ = sim.simxGetFloatSignal(self.clientID, 'accelX', sim.simx_opmode_streaming)
        ret[4], _ = sim.simxGetFloatSignal(self.clientID, 'accelY', sim.simx_opmode_streaming)
        ret[5], _ = sim.simxGetFloatSignal(self.clientID, 'accelZ', sim.simx_opmode_streaming)

        print("IMU return codes:")
        print(ret)

        ret, _ = sim.simxGetObjectPosition(self.clientID, self.quadHandle, -1, sim.simx_opmode_streaming)
        print("Position return code:")
        print(ret)

    def get_pos(self):
        err, pos = sim.simxGetObjectPosition(self.clientID, self.quadHandle, -1, sim.simx_opmode_buffer)
        print("Position feedback return code: ")
        print(err)
        return pos

    def get_imu(self):
        err = [0, 0, 0, 0, 0, 0]
        err[0], gX = sim.simxGetFloatSignal(self.clientID, 'gyroX', sim.simx_opmode_buffer)
        err[1], gY = sim.simxGetFloatSignal(self.clientID, 'gyroY', sim.simx_opmode_buffer)
        err[2], gZ = sim.simxGetFloatSignal(self.clientID, 'gyroZ', sim.simx_opmode_buffer)

        err[3], aX = sim.simxGetFloatSignal(self.clientID, 'accelX', sim.simx_opmode_buffer)
        err[4], aY = sim.simxGetFloatSignal(self.clientID, 'accelY', sim.simx_opmode_buffer)
        err[5], aZ = sim.simxGetFloatSignal(self.clientID, 'accelZ', sim.simx_opmode_buffer)

        print("IMU feedback return code: ")
        print(err)

        return [gX, gY, gZ, aX, aY, aZ]

    def init_prop(self):
        ret = [0, 0, 0, 0]
        for i in range(len(self.prop)):
            ret[i] = sim.simxClearFloatSignal(self.clientID, self.prop[i], sim.simx_opmode_oneshot)

        print("Prop initialization return code:")
        print(ret)

        # Set all propellers to zero
        self.move_prop(self.prop_vel)

    def move_prop(self, targetVel):
        ret = [0, 0, 0, 0]
        for i in range(len(self.prop)):
            ret[i] = sim.simxSetFloatSignal(self.clientID, self.prop[i], targetVel[i], sim.simx_opmode_oneshot)

        print("Prop move return code:")
        print(ret)

def test_01():
    session = Simulation()
    clientID = session.init()

    if clientID == -1:
        print('Failed connecting to remote API server')
        sys.exit()

    else:
        print('Connected to remote API server')
        quad = Quadcopter(clientID)
        quad.init()
        print("Quadcopter initialized")

        quad.move_prop([0, 2, 0, 2])
        time.sleep(5)
        quad.move_prop([2, 0, 2, 0])
        time.sleep(5)
        quad.move_prop([0, 0, 0, 0])


        session.close()
        print("Simulation Terminated")


if __name__ == "__main__":
    test_01()





