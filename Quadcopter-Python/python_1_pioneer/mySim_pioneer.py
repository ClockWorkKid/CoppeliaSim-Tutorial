# Code works with Pioneer_p3dx robot
# scene_pioneer.ttt
# simRemoteApi.start(19999)

import time
import sys
import sim


print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        print('Number of objects in the scene: ', len(objs))
    else:
        print('Remote API function call returned with error code: ', res)

    _, motorLeft = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
    _, motorRight = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

    sim.simxSetJointTargetVelocity(clientID, jointHandle=motorLeft, targetVelocity=1, operationMode=sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, jointHandle=motorRight, targetVelocity=1, operationMode=sim.simx_opmode_oneshot)

    time.sleep(2)

    sim.simxSetJointTargetVelocity(clientID, jointHandle=motorLeft, targetVelocity=0, operationMode=sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, jointHandle=motorRight, targetVelocity=0, operationMode=sim.simx_opmode_oneshot)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime = time.time()

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID, 'Hello CoppeliaSim!', sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
    sys.exit()

print('Program ended')
