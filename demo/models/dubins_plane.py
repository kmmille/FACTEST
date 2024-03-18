import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are, inv, solve_discrete_are
from math import sin, cos, tan, atan, pi, sqrt, ceil

class dubins_plane: # Based on the AUV tracking control
    def __init__(self) -> None:
        self.gamma = 1
        self.k2 = 0.001
        self.k3 = 0.001

        self.ref_traj = [[100,100,100,0,0]]
        self.ref_input = [[1,0,0,0]]

        self.dt = 0.1

        pass

    def dubinsDynamics(self, state, t, input):
        x,y,z,pitch,yaw = state
        v,wx,wy,wz = input

        xdot = v*cos(yaw)*cos(pitch)
        ydot = v*sin(yaw)*cos(pitch)
        zdot = v*sin(pitch)

        pitchdot = wy
        yawdot = wz*(1/cos(pitch))

        return [xdot, ydot, zdot, pitchdot, yawdot]

    def computeErr(self, state, ref_state):
        x,y,z,pitch,yaw = state

        xref,yref,zref,pitchref,yawref = ref_state

        R = np.array([[cos(yaw)*cos(pitch),-sin(yaw), cos(yaw)*sin(pitch)],
                      [sin(yaw)*cos(pitch), cos(yaw), sin(yaw)*sin(pitch)],
                      [-sin(pitch)        , 0       , cos(pitch)         ]])

        R_ref = np.array([[cos(yawref)*cos(pitchref),-sin(yawref), cos(yawref)*sin(pitchref)],
                          [sin(yawref)*cos(pitchref), cos(yawref), sin(yawref)*sin(pitchref)],
                          [-sin(pitchref)           , 0          , cos(pitchref)            ]])

        pos_err = np.matmul(np.transpose(R), np.array([xref-x, yref-y, zref-z]))
        x_err = pos_err[0]
        y_err = pos_err[1]
        z_err = pos_err[2]
        pitch_err = pitchref - pitch
        yaw_err = yawref - yaw

        return [x_err, y_err, z_err, pitch_err, yaw_err]

    def trackingControl(self, curr_state, ref_state, ref_input):
        x,y,z,pitch,yaw = curr_state

        x_err, y_err, z_err, pitch_err, yaw_err = self.computeErr(curr_state, ref_state)
        v_ref, wx_ref, wy_ref, wz_ref = ref_input

        v_b = v_ref*(cos(yaw_err)*cos(pitch_err) - 1) + (self.gamma**2)*x_err
        v = v_ref + v_b

        B = np.array([[1, 0, tan(pitch)],
                      [0, 1, 0         ],
                      [0, 0, 1/cos(pitch)]])
        B_ref = np.array([[1, 0, tan(pitch)],
                          [0, 1, 0         ],
                          [0, 0, 1/cos(pitch)]])

        q = np.array([0, (-z_err*v_ref)/self.k2, (y_err*v_ref*cos(pitch_err))/self.k3])
        p = np.array([0,self.k2*sin(pitch_err),self.k3*sin(yaw_err)])

        u_b = np.matmul(np.linalg.inv(B),(q + (np.matmul(B_ref-B, np.array([wx_ref, wy_ref, wz_ref]))) + p))
        u = np.array([wx_ref,wy_ref,wz_ref])+u_b
        
        w_x = u[0]
        w_y = u[1]
        w_z = u[2]

        return [v, w_x, w_y, w_z]

    def dubinsControlledDynamics(self, state, t):
        ref_idx = ceil(t/self.dt)
        if ref_idx >= len(self.ref_traj):
            ref_idx = len(self.ref_traj)-1

        ref_state = self.ref_traj[ref_idx]
        ref_input = self.ref_input[ref_idx]
        v, w_x, w_y, w_z = self.trackingControl(state, ref_state, ref_input)
        
        x,y,z,pitch,yaw = state

        xdot = v*cos(yaw)*cos(pitch)
        ydot = v*sin(yaw)*cos(pitch)
        zdot = v*sin(pitch)

        pitchdot = w_y
        yawdot = w_z*(1/cos(pitch))

        return [xdot, ydot, zdot, pitchdot, yawdot]

    def run_simulation(self, xref, vref, initial_state, T):
        time_array = np.arange(0,T,self.dt)
        # self.set_ref(xref, vref)
        state_trace = odeint(self.dubinsControlledDynamics, initial_state, time_array)
        return state_trace

    def errBound(self, init_poly, i):
        err_0 = init_poly.chebR
        err = sqrt(err_0**2 + (4*i)*(self.k2+self.k3))
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

            mz = p2[2] - p1[2]
            bz = p1[2]
            
            yaw_ref = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])
            pitch_ref = np.arctan2((np.array(p2) - np.array(p1))[2], sqrt(((np.array(p2) - np.array(p1))[0])**2 + ((np.array(p2) - np.array(p1))[1])**2))

            t = np.linalg.norm(np.array(p2)-np.array(p1))/vref

            while curr_time <= t + prev_t:
                px = mx*((curr_time - prev_t)/t) + bx
                py = my*((curr_time - prev_t)/t) + by
                pz = mz*((curr_time - prev_t)/t) + bz
                self.ref_traj.append((px,py,pz,pitch_ref,yaw_ref))
                self.ref_input.append((vref,0,0,0))
                curr_time += self.dt

            prev_t += t