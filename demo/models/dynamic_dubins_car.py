import numpy as np
from scipy.integrate import odeint
from math import sin, cos, tan, atan, pi, sqrt, ceil

class dynamic_planar_dubins:
    def __init__(self) -> None:
        ##################
        # Car parameters #
        ##################
        self.length = 1
        self.k = 100

        #########################
        # Controller parameters #
        #########################
        self.kappa = 10
        self.G = 100

        #######################################################
        # Reference state-input trajectories (For simulation) #
        #######################################################
        self.ref_traj = None
        self.ref_input = None

    def dynamics(self, state, t, input):
        x,y,heading,vel,turning = state
        force,torque = input

        xdot = vel*cos(heading)
        ydot = vel*sin(heading)
        headingdot = turning
        veldot = force - self.k*vel
        turningdot = torque - self.k*turning

        return [xdot, ydot, headingdot, veldot, turningdot]

    def trackingController(self, state, ref_state, ref_input):
        x,y,heading,vel,turning = state
        xref,yref,headingref,velref,turningref = ref_state

        forceref,torqueref = ref_input

        aref = (forceref - self.k*velref)

        zx = x + self.length*cos(heading) - (xref + self.length*cos(headingref))
        zy = y + self.length*sin(heading) - (yref + self.length*sin(headingref))

        vx = velref*cos(heading) - self.length*sin(headingref)*turningref
        vy = velref*sin(heading) + self.length*cos(headingref)*turningref

        ax = aref*cos(headingref)
        ay = aref*sin(headingref)

        gx = zx*(self.G/(1 + np.linalg.norm(np.array([zx, zy]))))
        gy = zy*(self.G/(1 + np.linalg.norm(np.array([zx, zy]))))

        ux = ax + self.k*vx - gx
        uy = ay + self.k*vy - gy

        force = cos(heading)*(ux + vel*turning*sin(heading)) + self.length*(turning**2)*cos(heading) + sin(heading)*(uy - vel*turning*cos(heading) + self.length*(turning**2)*sin(heading)) + self.length*turning**2
        torque = (-sin(heading)/self.length)*(ux + vel*turning*sin(heading) + self.length*(turning**2)*cos(heading)) + (cos(heading)/self.length)*(uy - vel*turning*cos(heading) + self.length*(turning**2)*sin(heading)) - (vel*turning/self.length)

        return [force, torque]

    def controlledDynamics(self, state, t):
        ref_idx = ceil(t/self.dt)
        if ref_idx >= len(self.ref_traj):
            ref_idx = len(self.ref_traj)-1

        ref_state = self.ref_traj[ref_idx]
        ref_input = self.ref_input[ref_idx]

        input = self.trackingController(state, ref_state, ref_input)
        xdot, ydot, headingdot, veldot, turningdot = self.dynamics(state, t, input)

        return [xdot, ydot, headingdot, veldot, turningdot]

    # TODO: Implement error bounds
    # TODO: Implement error dynamics
    # TODO: Implement TC Simulate for error dynamics (Can be used with DryVr)
    # TODO: Implement simulations (particularly dynamic-FACTEST simulation)
    
