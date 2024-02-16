import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are, inv, solve_discrete_are
from math import sin, cos, tan, atan, pi, sqrt


class dubins_car:
    def __init__(self) -> None:
        #############################
        # Tracking controller gains #
        #############################
        self.k1 = 100
        self.k2 = 100
        self.k3 = 100

        #######################################################
        # Reference state-input trajectories (For simulation) #
        #######################################################
        self.state_ref = None
        self.input_ref = None

        self.state_ref_traj = None
        self.input_ref_traj = None

        # ##########################
        # # Initializing car state #
        # ##########################
        # self.initial_state = initial_state 
        # # self.initial_mode = initial_mode
        # self.curr_state = initial_state # For simulation

    def dubinsDynamics(self, state, t, input):
        x,y,theta = state
        v, w = input

        xdot = v*cos(theta)
        ydot = v*sin(theta)
        thetadot = w
        
        return [xdot, ydot, thetadot]

    def trackingControl(self, curr_state, ref_state, ref_input):
        x,y,theta = curr_state
        xref,yref,thetaref = ref_state
        vref,wref = ref_input

        xerr = (xref - x)*cos(theta) + (yref - y)*sin(theta)
        yerr = (xref - x)*(-sin(theta)) + (yref - y)*cos(theta)
        thetaerr = thetaref - theta

        v = vref*cos(thetaerr) + self.k1*xerr
        w = wref + vref*(self.k2*yerr + self.k3*sin(thetaerr)) 

        input = [v, w]

        return input

    def errBound(self, init_poly, i):
        err_0 = init_poly.chebR
        err = sqrt(err_0**2 + (4*i)/(self.k2))
        # err = 0.1
        return err

    def simulate(self, mode, initial_state, time_horizon, time_step):
        time_array = np.arange(0, time_horizon+time_step, time_step)
        if self.state_ref == None and self.input_ref == None:
            # No controller used here!
            input = [0, 0]
        elif (self.state_ref == None and self.input_ref != None) or (self.state_ref != None and self.input_ref == None):
            # Either the state or reference trajectory is not defined
            raise Exception('Both state and input trajectories must be defined!')
        else:
            # This is where both state and reference trajectories are defined
            input = self.trackingControl(initial_state, self.state_ref, self.input_ref)

        state_trace = odeint(self.dubinsDynamics, initial_state, time_array, args=(input,))

        trace = []

        for i in range(len(time_array)):
            trace.append([time_array[i]]+list(state_trace[i]))

        return trace

    # def TC_simulate(self, mode, initialSet, time_horizon, time_step, map=None):
    #     #TODO: Implement TC simulate for reachability
    #     return None
