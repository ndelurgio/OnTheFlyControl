from eom_dynamics import *
import numpy as np
class vehicle():
    def __init__(self,mass,Ixx,Iyy,Izz,r_motor,K_T,K_M,K_Y):
        self.mass = mass
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.r_motor = r_motor
        self.K_T = K_T
        self.K_M = K_M
        self.K_Y = K_Y

        self.pos = np.matrix([[0,0,0]]).T
        self.vel = np.matrix([[0,0,0]]).T
        self.DCM = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        self.omega = np.matrix([[0,0,0]]).T
        self.a_prev = np.matrix([[0,0,0]]).T
        self.Cdot_prev = np.zeros([3,3])
        self.alpha_prev = np.matrix([[0,0,0]]).T

        self.dynamics = eom_dynamics(mass,Ixx,Iyy,Izz,r_motor,K_T,K_M,K_Y)

    def eom(self,dt, w1,w2,w3,w4):
        alpha = self.dynamics.getAngAccel(w1,w2,w3,w4,self.omega)
        self.omega = self.omega + dt/2*(self.alpha_prev + alpha)
        S_omega = np.matrix([[0, self.omega[2,0], -1*self.omega[1,0]],[-1*self.omega[2,0], 0, self.omega[0,0]],[self.omega[1,0], self.omega[0,0], 0]])
        Cdot = S_omega*self.DCM#np.dot(S_omega,self.DCM)
        # print(Cdot)
        self.DCM = self.DCM + dt/2*(self.Cdot_prev+Cdot)
        a = self.DCM*self.dynamics.getLinAccel(w1,w2,w3,w4) + np.matrix([[0,0,-9.806]]).T
        v = self.vel + dt/2*(self.a_prev+a)
        self.pos = self.pos + dt/2*(self.vel+v)
        self.vel = v
        self.alpha_prev = alpha
        self.Cdot_prev = Cdot
        self.a_prev = a
        self.v_prev = v
        return self.DCM, self.vel, self.pos