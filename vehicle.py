import numpy as np
class vehicle():
    def __init__(mass,Ixx,Iyy,Izz,r_motor,K_T,K_M,K_Y):
        self.mass = mass
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.r_motor = r_motor
        self.K_T = K_T
        self.K_M = K_M
        self.K_Y = K_Y

        self.pos = np.array([0,0,0])
        self.vel = np.array([0,0,0])
        self.dcm = np.array([1,0,0],[0,1,0],[0,0,1])
        self.bodyRates = np.array([0,0,0])


        self.dynamics = eom_dynamics() #figure out inheritance

    def lin_kinematics():
        a_x_b, a_y_b, a_z_b = self.dynamics.getLinAccel()
        #account for gravity
        #use dcm to rotate to inertial
    def ang_kinematics():
        #get body ang vels
        #use DCM_dot equation (inputs are previous dcm and current body rates)
        # integrate DCM_dot
        #    