from vehicle import *

dt = 3
mass=.08
Ixx = .01
Iyy = .01
Izz = .002
r_motor=.135
K_T = .024515
K_M= .0000625
K_Y = .0000625
quad = vehicle(1,1,1,1,1,K_T,K_M,K_Y)
for i in range(3):
    DCM, vel, pos = quad.eom(dt,9,11,11,9)
    print("DCM: " + str(DCM))
    print("Vel: " + str(vel))
    print("Pos: " + str(pos))
