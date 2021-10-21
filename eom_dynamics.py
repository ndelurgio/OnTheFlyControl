import numpy as np
class eom_dynamics():
    def __init__(self,mass,Ixx,Iyy,Izz,r_motor,K_T,K_M,K_Y):
        self.mass = mass
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.r_motor = r_motor
        self.K_T = K_T
        self.K_M = K_M
        self.K_Y = K_Y
        

    def getAngAccel(self,w1,w2,w3,w4,omega):
        wx = omega[0,0]
        wy = omega[1,0]
        wz = omega[2,0]
        Tx_b = self.r_motor*self.K_M*(w1**2-w2**2-w3**2+w4**2)
        Ty_b = self.r_motor*self.K_M*(w1**2+w2**2-w3**2-w4**2)
        Tz_b = self.K_Y*(w1**2-w2**2+w3**2-w4**2)
        wdotx_b = (Tx_b+(self.Iyy-self.Izz)*wy*wz)/self.Ixx 
        wdoty_b = (Ty_b+(self.Izz-self.Ixx)*wz*wx)/self.Iyy 
        wdotz_b = (Tz_b+(self.Ixx-self.Iyy)*wy*wx)/self.Izz
        return np.matrix([[wdotx_b, wdoty_b, wdotz_b]]).T
    def getLinAccel(self,w1,w2,w3,w4):
        Fx_b = 0
        Fy_b = 0
        Fz_b = self.K_T*(w1**2+w2**2+w3**2+w4**2)
        return np.matrix([[Fx_b/self.mass, Fy_b/self.mass, Fz_b/self.mass]]).T
    
    ## CONVENTION
    ## w1: upper-right prop, counter-clockwise
    ## w2: upper-left prop, clockwise
    ## w3: bottom-left prop, counter-clockwise
    ## w4: bottom-right prop, clockwise