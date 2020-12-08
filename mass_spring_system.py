import numpy as np
import matplotlib.pyplot as plt
from run import *

# A script containing all initial conditions of the system
# hyperparameter list:
    # damping_threshold
def mss(n, nz, L0_l):
    ############################################################################
    # General Parameters #
    ############################################################################
    # we have longitudinal resting length in meters
    L0_e = np.sqrt(2)*L0_l # edge resting length in meters
    L0_c = np.sqrt(3)*L0_l # corner resting length  in meters
    ############################################################################
    # Particles Variables #
    ############################################################################

    #Initialize initial position array (cartesian coordinates)
    Pos_init = np.zeros((3,nz,n,n))
    for n_i in range(n):
        Pos_init[0, :, :, n_i] = n_i * L0_l #x
        Pos_init[1, :, n_i, :] = n_i * L0_l #y
    for n_i in range(nz):
        Pos_init[2, n_i, :, :] = n_i * L0_l #z

    # Masses array
    M = np.ones((nz,n,n))
    # unbounded: make bordering masses 0.
    for i in range(len(M)):
        for j in range(len(M[i])):
            for k in range(len(M[i,j])):
                if i == 0 or i == nz-1 or j == 0 or j == n-1 or k == 0 or k == n-1:
                    M[i,j,k] = 0


    ############################################################################
    # Initial Kinematics #
    ############################################################################
    # Initialize initial acceleration and velocity arrays
    # n-2 dimension: border masses have no movement
    A_init = np.zeros((3,nz-2,n-2,n-2))
    V_init = np.zeros((3,nz-2,n-2,n-2))



    ############################################################################
    # SPRINGS VARIABLES AND FUNCTIONS  #
    ############################################################################

    # there are n-2 springs of each type, listed below.
    K = np.ones((26, nz-2, n-2, n-2))
    # unbounded: make bordering springs 0.
    # for n in range(len(K)):
    #     for i in range(len(K[n])):
    #         for j in range(len(K[n,i])):
    #             for k in range(len(K[n,i,j])):
    #                 if i == 0 or i == nz-3 or j == 0 or j == n-3 or k == 0 or k == n-3:
    #                     K[n,i,j,k] = 0
    # damping variable

    damping_constant_critical = np.sqrt(4*np.mean(M)*np.mean(k))

    ############################################################################
    # Randomization function #
    ############################################################################

    return L0_e, L0_c, Pos_init, M, A_init, V_init, K, damping_constant_critical
