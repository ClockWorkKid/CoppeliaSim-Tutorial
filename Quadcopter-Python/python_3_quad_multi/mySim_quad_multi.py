# Code works with quadrirotor model
# scene_quad_multi.ttt
# simRemoteApi.start(19999)

import sim
import time
import sys
import numpy as np
import math
from scipy.optimize import linear_sum_assignment

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
        self.ID = ''
        self.targetHandle = None
        self.pos = np.zeros((1, 3))
        self.target_pos = np.zeros((1, 3))

    def init(self, ID = ''):
        self.ID = ID
        if ID == '':
            err, self.quadHandle = sim.simxGetObjectHandle(self.clientID, 'Quadricopter', sim.simx_opmode_blocking)
        else:
            err, self.quadHandle = sim.simxGetObjectHandle(self.clientID, 'Quadricopter#'+ID, sim.simx_opmode_blocking)
        if err != 0:
            print("Quadricopter " + ID + " Initialization failed")

        if ID == '':
            err, self.targetHandle = sim.simxGetObjectHandle(self.clientID, 'Quadricopter_target', sim.simx_opmode_blocking)
        else:
            err, self.targetHandle = sim.simxGetObjectHandle(self.clientID, 'Quadricopter_target#'+ID, sim.simx_opmode_blocking)
        if err != 0:
            print("Target " + ID + " Initialization failed")

        # err, _ = sim.simxGetObjectPosition(self.clientID, self.targetHandle, -1, sim.simx_opmode_streaming)
        print('Initializing position for quadricopter ' + ID)
        err, self.target_pos = sim.simxGetObjectPosition(self.clientID, self.targetHandle, -1,
                                                             sim.simx_opmode_blocking)
        err = sim.simxSetObjectPosition(self.clientID, self.targetHandle, -1, self.target_pos, sim.simx_opmode_oneshot)

    def get_pos(self):
        err, self.pos = sim.simxGetObjectPosition(self.clientID, self.quadHandle, -1, sim.simx_opmode_blocking)
        if err != 0:
            print("Get Position " + self.ID + " failed")
        return self.pos

    def set_pos(self, target):
        self.target_pos = np.array(target)
        err = sim.simxSetObjectPosition(self.clientID, self.targetHandle, -1, self.target_pos, sim.simx_opmode_oneshot)
        if err != 0:
            print("Set target " + self.ID + " failed")

    def displacement(self, dist):
        self.target_pos = np.array(dist) + np.array(self.target_pos)
        err = sim.simxSetObjectPosition(self.clientID, self.targetHandle, -1, self.target_pos, sim.simx_opmode_oneshot)
        if err != 0:
            print("Set displacement " + self.ID + " failed")


def maneuver_square(quad, timeFactor):
    step_delay = 2
    num_quads = len(quad)

    for iteration in range(1):

        for quad_no in range(num_quads):
            quad[quad_no].displacement(dist=[0.3, 0, 0])
        time.sleep(step_delay / timeFactor)

        for quad_no in range(num_quads):
            quad[quad_no].displacement(dist=[0, 0.3, 0])
        time.sleep(step_delay / timeFactor)

        for quad_no in range(num_quads):
            quad[quad_no].displacement(dist=[-0.3, 0, 0])
        time.sleep(step_delay / timeFactor)

        for quad_no in range(num_quads):
            quad[quad_no].displacement(dist=[0, -0.3, 0])
        time.sleep(step_delay / timeFactor)

        print("Iteration " + str(iteration) + " complete")


def target_matching(target_pos, quad_pos):
    """
    Assigns detections to tracked object (both represented as bounding boxes)
    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    total_pos = len(target_pos)
    total_quad = len(quad_pos)

    cost_matrix = np.zeros((total_pos, total_quad), dtype=np.float32)

    for m in range(total_pos):
        for n in range(total_quad):
            cost_matrix[m, n] = np.linalg.norm(target_pos[m, :] - quad_pos[n, :])

    indices = np.transpose(np.asarray(linear_sum_assignment(cost_matrix)))

    return indices


def maneuver_circle_formation(quad, timeFactor):
    radius = 2
    steps = 50
    step_delay = 0.2
    num_quads = len(quad)

    target_points = np.zeros((num_quads, 2), dtype=float)
    quad_location = np.zeros_like(target_points)

    angle_step = 2*math.pi/num_quads
    for i in range(num_quads):
        theta = angle_step * i
        target_points[i, :] = (radius * math.cos(theta), radius * math.sin(theta))

    for i in range(num_quads):
        quad_pos = quad[i].get_pos()
        quad_location[i, :] = (quad_pos[0], quad_pos[1])

    matching_indices = target_matching(target_points, quad_location)
    print("Targets matched with nearest agent: ")
    print(np.transpose(matching_indices[:, 1]))

    # Calculate necessary tracjectory for each quad
    print("Calculating Trajectories")
    trajectory_list = np.zeros((num_quads, steps, 2), dtype=float)
    for target_no in range(num_quads):
        quad_no = matching_indices[target_no, 1]
        start_pos = quad_location[quad_no, :]
        end_pos = target_points[target_no, :]
        trajectory_list[quad_no, :, 0] = np.linspace(start_pos[0], end_pos[0], steps)
        trajectory_list[quad_no, :, 1] = np.linspace(start_pos[1], end_pos[1], steps)

    # Smooth transition of all quads in mini steps
    print("Attempting to move to positions")
    for step_no in range(steps):
        for quad_no in range(num_quads):
            current_target_pos = [trajectory_list[quad_no, step_no, 0],
                                  trajectory_list[quad_no, step_no, 1],
                                  quad[quad_no].target_pos[2]]
            quad[quad_no].set_pos(current_target_pos)
        time.sleep(step_delay/timeFactor)

    print("Circular formation complete")


def maneuver_orbit(quad, timeFactor):
    steps = 200
    step_delay = 0.125
    num_quads = len(quad)
    step_angle = 2 * math.pi / steps

    radius_list = np.zeros(num_quads, dtype=float)
    quad_location = np.zeros((num_quads, 2), dtype=float)

    for quad_no in range(num_quads):
        quad_pos = quad[quad_no].get_pos()
        quad_location[quad_no, :] = (quad_pos[0], quad_pos[1])
        radius_list[quad_no] = math.sqrt(quad_pos[0]**2 + quad_pos[1]**2)

    # Smooth transition of all quads in mini steps
    print("Orbiting around origin")
    for step_no in range(int(steps/4)):
        for quad_no in range(num_quads):
            r = radius_list[quad_no]
            x, y = quad_location[quad_no, 0], quad_location[quad_no, 1]
            theta = math.atan2(y, x)
            theta = theta + step_angle
            x, y = r * math.cos(theta), r * math.sin(theta)
            quad_location[quad_no, 0], quad_location[quad_no, 1] = x, y

            current_target_pos = [x, y, quad[quad_no].target_pos[2]]
            quad[quad_no].set_pos(current_target_pos)

        time.sleep(step_delay/timeFactor)

    print("quarter orbit complete")


if __name__ == "__main__":

    session = Simulation()
    clientID = session.init()

    timeFactor = 0.65
    total_quads = 16
    quad = []

    if clientID == -1:
        print('Failed connecting to remote API server')
        sys.exit()

    else:
        print('Connected to remote API server')
        # initialize the quadcopter objects and connect to simulation handles
        for i in range(total_quads):
            quad.append(Quadcopter(clientID))
            if i == 0:
                ID = ''
            else:
                ID = str(i-1)
            quad[i].init(ID)
        print("Quadcopters initialized")

        maneuver_square(quad, timeFactor)
        time.sleep(2 / timeFactor)
        maneuver_orbit(quad, timeFactor)
        time.sleep(2/timeFactor)
        maneuver_circle_formation(quad, timeFactor)
        time.sleep(2/timeFactor)
        maneuver_orbit(quad, timeFactor)

        session.close()
        print("Simulation Terminated")
