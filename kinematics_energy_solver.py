from hookes_solver_functions import *
from energy_solver_functions import *
import numpy as np

# hyperparameter list:
    #
###################################################################################################
# Initial Position, Velocity, Acceleration Matrices Initializer  #
###################################################################################################
# Processes initial conditions and inserts them into an array with a time dimension, separated by direction
def initialize_kinematics(T, n, Pos_init, A_init, V_init):

    Pos_x, Pos_y, Pos_z = np.zeros((np.shape(T)[0], n, n, n)), np.zeros((np.shape(T)[0], n, n, n)), np.zeros((np.shape(T)[0], n, n, n))
    A_x, A_y, A_z = np.zeros((np.shape(T)[0], n-2, n-2, n-2)), np.zeros((np.shape(T)[0], n-2, n-2, n-2)), np.zeros((np.shape(T)[0], n-2, n-2, n-2))
    V_x, V_y, V_z = np.zeros((np.shape(T)[0], n-2, n-2, n-2)), np.zeros((np.shape(T)[0], n-2, n-2, n-2)), np.zeros((np.shape(T)[0], n-2, n-2, n-2))

    Pos_x[:], Pos_y[:], Pos_z[:] = Pos_init[0],Pos_init[1],Pos_init[2]
    A_x[0], A_y[0], A_z[0] = A_init[0], A_init[1], A_init[2] #Set initial accelerations and velocities
    V_x[0], V_y[0], V_z[0] = V_init[0], V_init[1], V_init[2]

    return  Pos_x, Pos_y, Pos_z, A_x, A_y, A_z, V_x, V_y, V_z

###################################################################################################
# Net Force Calculator
###################################################################################################

def net_force_calculator(Pos_init, V_init, damping_constant, k,L0_l, L0_c, L0_e, M):

        ############################################################################
        # Process Particles Positions #
        ############################################################################
        # Current Particle  #
        particle = Pos_init[:, 1:-1, 1:-1, 1:-1]

        # 6 Longitudinal Particles #

        particle_xp = Pos_init[:, 1:-1, 1:-1, 2:]
        particle_xn = Pos_init[:, 1:-1, 1:-1, 0:-2]
        particle_yp = Pos_init[:, 1:-1,2:,1:-1]
        particle_yn = Pos_init[:, 1:-1,0:-2,1:-1]
        particle_zp = Pos_init[:, 2:, 1:-1, 1:-1]
        particle_zn = Pos_init[:, 0:-2, 1:-1, 1:-1]

        # 12 Edge Particles #
        particle_xp_yp = Pos_init[:, 1:-1, 2:, 2:]
        particle_xp_yn = Pos_init[:, 1:-1, 0:-2, 2:]
        particle_xp_zp = Pos_init[:, 2:, 1:-1, 2:]
        particle_xp_zn = Pos_init[:, 0:-2, 1:-1, 2:]
        particle_xn_yp = Pos_init[:, 1:-1, 2:, 0:-2]
        particle_xn_yn = Pos_init[:, 1:-1, 0:-2, 0:-2]
        particle_xn_zp = Pos_init[:, 2:, 1:-1, 0:-2]
        particle_xn_zn = Pos_init[:, 0:-2, 1:-1, 0:-2]
        particle_yp_zp = Pos_init[:, 2:, 2:,1:-1]
        particle_yp_zn = Pos_init[:, 0:-2, 2:, 1:-1]
        particle_yn_zp = Pos_init[:, 2:, 0:-2, 1:-1]
        particle_yn_zn  = Pos_init[:, 0:-2, 0:-2, 1:-1]

        # 8 Corner Particles #
        particle_xp_yp_zp  = Pos_init[:, 2:, 2:, 2:]
        particle_xp_yp_zn = Pos_init[:, 0:-2, 2:, 2:]
        particle_xp_yn_zp = Pos_init[:, 2:, 0:-2, 2:]
        particle_xp_yn_zn = Pos_init[:, 0:-2, 0:-2, 2:]
        particle_xn_yp_zp = Pos_init[:, 2:, 2:, 0:-2]
        particle_xn_yp_zn = Pos_init[:, 0:-2, 2:, 0:-2]
        particle_xn_yn_zp = Pos_init[:, 2:, 0:-2, 0:-2]
        particle_xn_yn_zn = Pos_init[:, 0:-2, 0:-2, 0:-2]

        ############################################################################
        # Process Springs Matrix #
        ############################################################################

        # 6 Longitudinal Springs #
        k_xp = k[0]
        k_xn = k[1]
        k_yp = k[2]
        k_yn = k[3]
        k_zp = k[4]
        k_zn = k[5]

        # 12 Edge Springs #
        k_xp_yp = k[6]
        k_xp_yn = k[7]
        k_xp_zp = k[8]
        k_xp_zn = k[9]
        k_xn_yp = k[10]
        k_xn_yn = k[11]
        k_xn_zp = k[12]
        k_xn_zn = k[13]
        k_yp_zp = k[14]
        k_yp_zn = k[15]
        k_yn_zp = k[16]
        k_yn_zn  = k[17]

        # 8 Corner Springs #
        k_xp_yp_zp  = k[18]
        k_xp_yp_zn = k[19]
        k_xp_yn_zp = k[20]
        k_xp_yn_zn = k[21]
        k_xn_yp_zp = k[22]
        k_xn_yp_zn = k[23]
        k_xn_yn_zp = k[24]
        k_xn_yn_zn = k[25]
        # force components
        # 6 longitudinal
        length_deviation_xp, force_xp_x, force_xp_y, force_xp_z = hookes_force_solver(particle, particle_xp, L0_l, k_xp)
        length_deviation_xn, force_xn_x, force_xn_y, force_xn_z = hookes_force_solver(particle, particle_xn, L0_l, k_xn)
        length_deviation_yp, force_yp_x, force_yp_y, force_yp_z = hookes_force_solver(particle, particle_yp, L0_l, k_yp)
        length_deviation_yn, force_yn_x, force_yn_y, force_yn_z = hookes_force_solver(particle, particle_yn, L0_l, k_yn)
        length_deviation_zp, force_zp_x, force_zp_y, force_zp_z = hookes_force_solver(particle, particle_zp, L0_l, k_zp)
        length_deviation_zn, force_zn_x, force_zn_y, force_zn_z = hookes_force_solver(particle, particle_xp, L0_l, k_zn)

        # 12 edge
        length_deviation_xp_yp, force_xp_yp_x , force_xp_yp_y , force_xp_yp_z  = hookes_force_solver(particle, particle_xp_yp , L0_e, k_xp_yp)
        length_deviation_xp_yn, force_xp_yn_x , force_xp_yn_y , force_xp_yn_z  = hookes_force_solver(particle, particle_xp_yn , L0_e, k_xp_yn)
        length_deviation_xp_zp, force_xp_zp_x , force_xp_zp_y , force_xp_zp_z  = hookes_force_solver(particle, particle_xp_zp , L0_e, k_xp_zp)
        length_deviation_xp_zn, force_xp_zn_x  , force_xp_zn_y , force_xp_zn_z  = hookes_force_solver(particle, particle_xp_zn , L0_e, k_xp_zn)
        length_deviation_xn_yp, force_xn_yp_x , force_xn_yp_y , force_xn_yp_z  = hookes_force_solver(particle, particle_xn_yp , L0_e, k_xn_yp)
        length_deviation_xn_yn, force_xn_yn_x , force_xn_yn_y , force_xn_yn_z  = hookes_force_solver(particle, particle_xn_yn , L0_e, k_xn_yn)
        length_deviation_xn_zp, force_xn_zp_x , force_xn_zp_y , force_xn_zp_z  = hookes_force_solver(particle, particle_xn_zp , L0_e, k_xn_zp)
        length_deviation_xn_zn, force_xn_zn_x , force_xn_zn_y , force_xn_zn_z  = hookes_force_solver(particle, particle_xn_zn , L0_e, k_xn_zn)
        length_deviation_yp_zp, force_yp_zp_x , force_yp_zp_y , force_yp_zp_z  = hookes_force_solver(particle, particle_yp_zp , L0_e, k_yp_zp)
        length_deviation_yp_zn, force_yp_zn_x , force_yp_zn_y , force_yp_zn_z  = hookes_force_solver(particle, particle_yp_zn , L0_e, k_yp_zn)
        length_deviation_yn_zp, force_yn_zp_x , force_yn_zp_y , force_yn_zp_z  = hookes_force_solver(particle, particle_yn_zp , L0_e, k_yn_zp)
        length_deviation_yn_zn, force_yn_zn_x , force_yn_zn_y, force_yn_zn_z  = hookes_force_solver(particle, particle_yn_zn , L0_e, k_yn_zn)

        # 8 corner
        length_deviation_xp_yp_zp, force_xp_yp_zp_x , force_xp_yp_zp_y , force_xp_yp_zp_z \
          = hookes_force_solver(particle, particle_xp_yp_zp , L0_c, k_xp_yp_zp)
        length_deviation_xp_yp_zn, force_xp_yp_zn_x , force_xp_yp_zn_y, force_xp_yp_zn_z \
          = hookes_force_solver(particle, particle_xp_yp_zn , L0_c, k_xp_yp_zn)
        length_deviation_xp_yn_zp, force_xp_yn_zp_x , force_xp_yn_zp_y , force_xp_yn_zp_z \
          = hookes_force_solver(particle, particle_xp_yn_zp , L0_c, k_xp_yn_zp)
        length_deviation_xp_yn_zn, force_xp_yn_zn_x , force_xp_yn_zn_y , force_xp_yn_zn_z \
          = hookes_force_solver(particle, particle_xp_yn_zn , L0_c, k_xp_yn_zn)
        length_deviation_xn_yp_zp, force_xn_yp_zp_x , force_xn_yp_zp_y , force_xn_yp_zp_z \
          = hookes_force_solver(particle, particle_xn_yp_zp , L0_c, k_xn_yp_zp)
        length_deviation_xn_yp_zn, force_xn_yp_zn_x , force_xn_yp_zn_y , force_xn_yp_zn_z \
          = hookes_force_solver(particle, particle_xn_yp_zn , L0_c, k_xn_yp_zn)
        length_deviation_xn_yn_zp, force_xn_yn_zp_x , force_xn_yn_zp_y , force_xn_yn_zp_z \
          = hookes_force_solver(particle, particle_xn_yn_zp , L0_c, k_xn_yn_zp)
        length_deviation_xn_yn_zn, force_xn_yn_zn_x , force_xn_yn_zn_y , force_xn_yn_zn_z \
          = hookes_force_solver(particle, particle_xn_yn_zn , L0_c, k_xn_yn_zn)

        # Calculate hookes force
        F_x_net_hookes = force_xp_x + force_xn_x + force_yp_x + force_yn_x + force_yn_x \
        + force_zp_x + force_zn_x + force_xp_yp_x + force_xp_yn_x + force_xp_zp_x + \
        force_xp_zn_x + force_xn_yp_x + force_xn_yn_x + force_xn_zp_x + force_xn_zn_x + \
        force_yp_zp_x + force_yp_zn_x + force_yn_zp_x + force_yn_zn_x + force_xp_yp_zp_x + \
        force_xp_yp_zn_x + force_xp_yn_zp_x + force_xp_yn_zn_x + force_xn_yp_zp_x + force_xn_yp_zn_x + \
        force_xn_yn_zp_x + force_xn_yn_zn_x

        F_y_net_hookes = force_xp_y + force_xn_y + force_yp_y + force_yn_y + force_yn_y \
        + force_zp_y + force_zn_y + force_xp_yp_y + force_xp_yn_y + force_xp_zp_y + \
        force_xp_zn_y + force_xn_yp_y + force_xn_yn_y + force_xn_zp_y + force_xn_zn_y + \
        force_yp_zp_y + force_yp_zn_y + force_yn_zp_y + force_yn_zn_y + force_xp_yp_zp_y + \
        force_xp_yp_zn_y + force_xp_yn_zp_y + force_xp_yn_zn_y + force_xn_yp_zp_y + force_xn_yp_zn_y + \
        force_xn_yn_zp_y + force_xn_yn_zn_y

        F_z_net_hookes = force_xp_z + force_xn_z + force_yp_z + force_yn_z + force_yn_z \
        + force_zp_z + force_zn_z + force_xp_yp_z + force_xp_yn_z + force_xp_zp_z + \
        force_xp_zn_z + force_xn_yp_z + force_xn_yn_z + force_xn_zp_z + force_xn_zn_z + \
        force_yp_zp_z + force_yp_zn_z + force_yn_zp_z + force_yn_zn_z + force_xp_yp_zp_z + \
        force_xp_yp_zn_z + force_xp_yn_zp_z + force_xp_yn_zn_z + force_xn_yp_zp_z + force_xn_yp_zn_z + \
        force_xn_yn_zp_z + force_xn_yn_zn_z

        # Calculate damping forcer
        F_x_net_damping = -((damping_constant*V_init[0]))
        F_y_net_damping = -((damping_constant*V_init[1]))
        F_z_net_damping = -((damping_constant*V_init[2]))

        F_x_net = F_x_net_hookes + F_x_net_damping
        F_y_net = F_y_net_hookes + F_y_net_damping
        F_z_net = F_z_net_hookes + F_z_net_damping

        length_deviation_list = [length_deviation_xp, length_deviation_xn, length_deviation_yp, length_deviation_yn, length_deviation_zp, length_deviation_zn, \
        length_deviation_xp_yp, length_deviation_xp_yn, length_deviation_xp_zp, length_deviation_xp_zn, length_deviation_xn_yp, length_deviation_xn_yn, \
        length_deviation_xn_zp, length_deviation_xn_zn, length_deviation_yp_zp, length_deviation_yp_zn, length_deviation_yn_zp, length_deviation_yn_zn, \
        length_deviation_xp_yp_zp, length_deviation_xp_yp_zn, length_deviation_xp_yn_zp, length_deviation_xp_yn_zn, length_deviation_xn_yp_zp, \
        length_deviation_xn_yp_zn,length_deviation_xn_yn_zp,length_deviation_xn_yn_zn]

        return F_x_net, F_y_net, F_z_net, length_deviation_list

###################################################################################################
# Kinamtics Calculator
###################################################################################################

def kinematics_calculator(Pos_init, A_init, V_init, k, n, L0_l, L0_c, L0_e, M, T, damping_constant):

    Pos_x, Pos_y, Pos_z, A_x, A_y, A_z, V_x, V_y, V_z = initialize_kinematics(T, n, Pos_init, A_init, V_init)

    # Initiate total energy array, ke array
    total_energy = np.zeros((np.shape(V_x)))
    pe_array = np.zeros((np.shape(total_energy)))
    ke_array = np.zeros((np.shape(total_energy)))

    # dt
    dt = T[1]-T[0]

    for t in range(len(T)-1):
        if T[t]%0.5 == 0:
            print('time: ',  T[t])

        F_x_net, F_y_net, F_z_net, length_deviation_list = net_force_calculator(Pos_init, V_init, damping_constant, k,L0_l, L0_c, L0_e, M)

        # Calculate Acceleration
        A_x[t+1] = F_x_net * 1 / (M[1:-1, 1:-1, 1:-1])
        A_y[t+1] = F_y_net * 1 / (M[1:-1, 1:-1, 1:-1])
        A_z[t+1] = F_z_net * 1 / (M[1:-1, 1:-1, 1:-1])


        # Calculate Velocity
        V_x[t+1] = V_x[t] + A_x[t] * dt
        V_y[t+1] = V_y[t] + A_y[t] * dt
        V_z[t+1] = V_z[t] + A_z[t] * dt


        #Calculate Position
        Pos_x[t+1] = Pos_x[t]
        Pos_y[t+1] = Pos_y[t]
        Pos_z[t+1] = Pos_z[t]



        Pos_x[t+1,1:-1,1:-1,1:-1] += (V_x[t]+V_x[t+1])/2 * dt
        Pos_y[t+1,1:-1,1:-1,1:-1] += (V_y[t]+V_y[t+1])/2 * dt
        Pos_z[t+1,1:-1,1:-1,1:-1] += (V_z[t]+V_z[t+1])/2 * dt


        ########################################################################
        #Update energies of each mass as matrices#
        ########################################################################
        #Update Kinetic Energy
        ke = calculate_kinetic_energy(M, V_x[t], V_y[t], V_z[t])
        #Update Potential Energy
        pe = calculate_potential_energy(k, length_deviation_list)

        # time arrays:
        total_energy[t] = ke + pe
        pe_array[t] = pe
        ke_array[t] = ke

        # Update Pos_init
        Pos_init = np.array((Pos_x[t], Pos_y[t],Pos_z[t]))
        V_init = np.array((V_x[t], V_y[t],V_z[t]))


    ############################################################################
    # OUTPUTS #
    ############################################################################
    # Pack kinematics matrices
    cartesian_kinematics_matrices = [Pos_x, Pos_y, Pos_z, A_x, A_y, A_z, V_x, V_y, V_z]

    return cartesian_kinematics_matrices, total_energy, pe_array, ke_array
