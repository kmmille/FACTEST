import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are, inv, solve_discrete_are
from math import sin, cos, tan, atan, pi, sqrt, ceil

class dubins_car:
    def __init__(self) -> None:
        #############################
        # Tracking controller gains #
        #############################
        self.k1 = 1000
        self.k2 = 1000
        self.k3 = 1000

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

        self.dt = 0.01

    def dubinsDynamics(self, state, t, input):
        x,y,theta = state
        v, w = input

        xdot = v*cos(theta)
        ydot = v*sin(theta)
        thetadot = w
        
        return [xdot, ydot, thetadot]

    def dubinsControlledDynamics(self, state, t):
        ref_idx = ceil(t/self.dt)
        if ref_idx >= len(self.ref_traj):
            ref_idx = len(self.ref_traj)-1

        ref_state = self.ref_traj[ref_idx]
        ref_input = self.ref_input[ref_idx]
        v,w = self.trackingControl(state, ref_state, ref_input)
        
        x,y,theta = state
        # v, w = input

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

        v = vref*cos(thetaerr) + self.k1/1000*xerr
        w = wref + vref*(self.k2/1000*yerr + self.k3/1000*sin(thetaerr)) 

        input = [v, w]

        return input

    def errBound(self, init_poly, i):
        err_0 = init_poly.chebR
        err = sqrt(err_0**2 + (4*i)/(self.k2))
        # err = 0.1
        return err

    def set_ref(self, xref, vref):
        self.ref_traj = []
        self.ref_input = []

        curr_time = 0
        prev_t = 0
        for i in range(len(xref)-1):
            p1 = xref[i]
            p2 = xref[i+1]

            mx = p2[0] - p1[0]
            bx = p1[0]
            
            my = p2[1] - p1[1]
            by = p1[1]
            
            theta_ref = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])

            t = np.linalg.norm(np.array(p2)-np.array(p1))/vref

            while curr_time <= t + prev_t:
                px = mx*((curr_time - prev_t)/t) + bx
                py = my*((curr_time - prev_t)/t) + by
                self.ref_traj.append((px,py,theta_ref))
                self.ref_input.append((vref,0))
                curr_time += self.dt

            prev_t += t

        return None

    def set_timed_ref(self, xref):
        self.ref_traj = []
        self.ref_input = []

        curr_time = 0
        prev_t = 0
        for i in range(len(xref)-1):
            p1 = xref[i]
            p2 = xref[i+1]

            mx = p2[0] - p1[0]
            bx = p1[0]
            
            my = p2[1] - p1[1]
            by = p1[1]
            
            theta_ref = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])

            t = p2[2] - p1[2]
            vref = np.linalg.norm(np.array(p2) - np.array(p1))/t

            while curr_time <= t + prev_t:
                px = mx*((curr_time - prev_t)/t) + bx
                py = my*((curr_time - prev_t)/t) + by
                self.ref_traj.append((px,py,theta_ref))
                self.ref_input.append((vref,0))
                curr_time += self.dt

            prev_t += t
            
        return None

    def run_simulation(self, xref, initial_state, T, vref = 1, sim_type = "base"): #TODO: MAY WANT TO COME UP WITH A BETTER WAY TO DO THIS
        if sim_type == "base":
            self.set_ref(xref, vref)
        else:
            self.set_timed_ref(xref)

        time_array = np.arange(0,T,self.dt)
        state_trace = odeint(self.dubinsControlledDynamics, initial_state, time_array)
        return state_trace

    def run_omega_simulation(self, hybrid_aut, curr_state, vref = 1, num_cycles = 3):
        if len(hybrid_aut.buchi_inits) > 1:
            raise Exception('Runs not implemented for automata with multiple possible initial states!')


        curr_buchi_state = hybrid_aut.buchi_inits[0]

        prefix_run = hybrid_aut.buchi_run['prefix']
        cycle_run = hybrid_aut.buchi_run['cycle']

        all_states = [] #TODO: MAY WANT TO SEPERATE CYCLES

        for transition in prefix_run:
            print('curr buchi state is ', curr_buchi_state)
            print('prefix transition is ', transition)
            possible_transitions = hybrid_aut.buchi_transitions[curr_buchi_state]

            possible_flows = hybrid_aut.flows[curr_buchi_state][str(transition)]
            found_flow = False
            for potential_flow in possible_flows:
                init_part = potential_flow['init']
                if init_part.contains(np.array([[curr_state[0]],[curr_state[1]]]))[0] and not found_flow:
                    waypoints = potential_flow['xref']
                    found_flow = True

            print('running simulation from ', curr_state)
            length = 0
            for i in range(1,len(waypoints)):
                length += np.linalg.norm(np.array(waypoints[i] - np.array(waypoints[i-1])))
            
            T = length/vref
            
            if length > 0:
                states = self.run_simulation(waypoints, curr_state, T, vref=vref)
                curr_state = states[-1]
                all_states.extend(states)

            for potential_transition in possible_transitions:
                if potential_transition[0] == transition:
                    #TODO: NEED TO USE THE JUMP FUNCTIONS HERE TO RUN A CHECK
                    print('updating buchi state to ', potential_transition[1])
                    curr_buchi_state = potential_transition[1]

        curr_cycle = 1
        while curr_cycle <= num_cycles:
            for transition in cycle_run:
                print('curr buchi state is ', curr_buchi_state)
                print('cycle ', curr_cycle,' transition is ', transition)
                possible_transitions = hybrid_aut.buchi_transitions[curr_buchi_state]

                possible_flows = hybrid_aut.flows[curr_buchi_state][str(transition)]
                found_flow = False
                for potential_flow in possible_flows:
                    init_part = potential_flow['init']
                    if init_part.contains(np.array([[curr_state[0]],[curr_state[1]]]))[0] and not found_flow:
                        waypoints = potential_flow['xref']
                        found_flow = True

                print('running simulation from ', curr_state)
                length = 0
                for i in range(1,len(waypoints)):
                    length += np.linalg.norm(np.array(waypoints[i] - np.array(waypoints[i-1])))
                
                T = length/vref
                
                if length > 0: #TODO: NEED TO FIGURE OUT WHAT TO DO WHEN THE LENGTH IS 0
                    states = self.run_simulation(waypoints, curr_state, T, vref=vref)
                    curr_state = states[-1]
                    all_states.extend(states)

                for potential_transition in possible_transitions:
                    if potential_transition[0] == transition:
                        #TODO: NEED TO USE THE JUMP FUNCTIONS HERE TO RUN A CHECK
                        print('updating buchi state to ', potential_transition[1])
                        curr_buchi_state = potential_transition[1]
            curr_cycle += 1
        
        return all_states

    #################################################################################################
    # Following code is in case we want to run reachability analysis on the synthesized controllers #
    #################################################################################################
    # def simulate(self, mode, initial_state, time_horizon, time_step):
    #     time_array = np.arange(0, time_horizon+time_step, time_step)
    #     if self.state_ref == None and self.input_ref == None:
    #         # No controller used here!
    #         input = [0, 0]
    #     elif (self.state_ref == None and self.input_ref != None) or (self.state_ref != None and self.input_ref == None):
    #         # Either the state or reference trajectory is not defined
    #         raise Exception('Both state and input trajectories must be defined!')
    #     else:
    #         # This is where both state and reference trajectories are defined
    #         input = self.trackingControl(initial_state, self.state_ref, self.input_ref)
    #     state_trace = odeint(self.dubinsDynamics, initial_state, time_array, args=(input,))
    #     trace = []
    #     for i in range(len(time_array)):
    #         trace.append([time_array[i]]+list(state_trace[i]))
    #     return trace

    # def TC_simulate(self, mode, initialSet, time_horizon, time_step, map=None):
    #     #TODO: Implement TC simulate for reachability
    #     return None
