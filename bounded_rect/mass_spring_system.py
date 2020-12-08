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
    k = np.ones((26, nz-2, n-2, n-2))

    # damping variable

    damping_constant_critical = np.sqrt(4*np.mean(M)*np.mean(k))

    ############################################################################
    # Randomization function #
    ############################################################################

    return L0_e, L0_c, Pos_init, M, A_init, V_init, k, damping_constant_critical
