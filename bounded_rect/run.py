from kinematics_energy_solver import *
from wave_tracker import *
from mass_spring_system import *
from stiffness_patterns import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import os
import matplotlib.animation as animation
from matplotlib import colors, cm
import json
import os, getopt, sys

# clarification: for kinematics matrices, dimension is (z,y,x)
def usage():
    script = os.path.basename(__file__)
    print("\n\nUsage:  " + script + " [options] <output directory>")
    print('''
                    Options:
                    -h --help (help)
                    -d --date
                    -t --timelimit (seconds)
                    -s --massperturbed (mass given a velocity, default is the middle mass on the xy plane)
                    -v --initialvelocity (initial velocity; e.g. 3)
                    -n --nxyplane (number of particles in a row; e.g. 10)
                    -z --nz (number of particles in a row; e.g. 10)
                    -f --fileindex (for mass experiments)
                    -e --eqlength (longitudinal; meters)
                    -l --numlayers (k value)
                    -i --highval (for k value )
                    -j --lowval (for k value)
                    -p --dampingthreshold (a weight value on critical linear damping constant)
                    -a --animation
                    <folder directory> (e.g. 3D_Wave_Data)

                    ''')
    sys.exit()

def main():
    opts, files = getopt.getopt(sys.argv[1:], "h:d:t:s:v:n:z:f:e:l:i:j:p:a:", [ "help", "date", "timelimit", "massperturbed", "initialvelocity", "nxyplane","nz", "fileindex", "eqlength", "numlayers" , "highval", "lowval", "dampingthreshold", "anim"])

    if len(files) != 1:
        usage()

    # defaults:
    parameters = {}
    parameters['d'] = 'nodate_'
    parameters['t'] = 5
    parameters['n'] = 3
    parameters['z'] = 3
    parameters['s'] = [0,0,0]
    parameters['v'] = 0
    parameters['f'] = 0
    parameters['e'] = 1
    parameters['l'] = 1
    parameters['i'] = 1
    parameters['j'] = 1
    parameters['p'] = 1
    parameters['a'] = 0 # by default, do not animate.

    # loop over options:
    for option, argument in opts:
        if option in ("-h", "--help"):
            usage()
        elif option in ("-d", "--date"):
            parameters['d'] = argument
        elif option in ("-t", "--time_limit"):
            parameters['t'] = argument
        elif option in ("-s", "--massperturbed"):
            parameters['s'] = argument
        elif option in ("-v", "--initialvelocity"):
            parameters['v'] = argument
        elif option in ("-n", "--nxyplane"):
            parameters['n'] = argument
        elif option in ("-z", "--nz"):
            parameters['z'] = argument
        elif option in ("-f", "--fileindex"):
            parameters['f'] = argument
        elif option in ("-e", "--eqlength"):
            parameters['e'] = argument
        elif option in ("-l", "--numlayers"):
            parameters['l'] = argument
        elif option in ("-i", "--highval"):
            parameters['i'] = argument
        elif option in ("-j", "--lowval"):
            parameters['j'] = argument
        elif option in ("-p", "--dampingthreshold"):
            parameters['p'] = argument
        elif option in ("-a", "--animation"):
            parameters['a'] = argument

    base = os.path.basename(files[0])
    input_video_name, input_video_extension = os.path.splitext(base)


    # parameters
    date = parameters['d']
    time_limit = float(parameters['t'])
    initial_velocity = float(parameters['v']) # in m/s
    n = int(parameters['n'])
    nz = int(parameters['z'])
    file_index = parameters['f']
    L0_l = float(parameters['e'])
    num_layers = int(parameters['l'])
    high_val = float(parameters['i'])
    low_val = float(parameters['j'])
    damping_threshold = float(parameters['p'])
    anim = int(parameters['a'])

    # set file paths
    folder_dir = files[0]


    # set up mass spring system
    L0_e, L0_c, Pos_init, M, A_init, V_init, k, damping_constant_critical = mss(n,nz, L0_l)
    # produce disorder
    k,  k_avg_val = z_layers(k, num_layers, low_val, high_val)

    # set initial conditions
    time_interval = np.arange(0, time_limit, 0.01) #s
    damping_constant = damping_constant_critical * damping_threshold
    start_position =  [int(nz/2-0.5),int(n/2-0.5),int(n/2-0.5)] # middle mass

    # if len(start_position) == 1:
    #     V_init[2,start_position[0]] = initial_velocity # z, hard coded direction for now...
    # elif len(start_position) == 2:
    #     V_init[2,start_position[0], start_position[1]] = initial_velocity # z, hard coded direction for now...
    # elif len(start_position) == 3:
    #     V_init[2,start_position[0], start_position[1], start_position[2]] = initial_velocity # z, hard coded direction for now...

    # ### temporarily switched to initial acceleration ### #
    if len(start_position) == 1:
        A_init[2,start_position[0]] = initial_velocity # z, hard coded direction for now...
    elif len(start_position) == 2:
        A_init[2,start_position[0], start_position[1]] = initial_velocity # z, hard coded direction for now...
    elif len(start_position) == 3:
        A_init[2,start_position[0], start_position[1], start_position[2]] = initial_velocity # z, hard coded direction for now...

    ###################################################################################################
    # Calculate position and energy propagation #
    ###################################################################################################
    # Pos_x, Pos_y, Pos_z, A_x, A_y, A_z, V_x, V_y, V_z, total_energy =  \
    cartesian_kinematics_matrices, total_energy, pe_array, ke_array,total_energy_1d = kinematics_calculator(Pos_init, A_init, V_init, k, n, nz, L0_l, L0_c, L0_e, M, time_interval, damping_constant)
    Pos_x, Pos_y, Pos_z, A_x, A_y, A_z, V_x, V_y, V_z = cartesian_kinematics_matrices

    ###################################################################################################
    # Plot Kinematics Plot (Movie) #
    ###################################################################################################
    if anim == 1:
        x_plot = Pos_x[:, 1:-1, 1:-1, 1:-1]
        y_plot = Pos_y[:, 1:-1, 1:-1, 1:-1]
        z_plot = Pos_z[:, 1:-1, 1:-1, 1:-1]

        fps = 20
        frames = np.arange(0, len(time_interval), fps)
        fig = plt.figure()
        ax = Axes3D(fig)


        def update(ts, x, y, z, total_energy):
            fig.clear()
            ax = Axes3D(fig)
            # ax.cla()
            ax.set_xlim3d(0, (n)*L0_l)
            ax.set_ylim3d(0, (n)*L0_l)
            ax.set_zlim3d(0, (n_z)*L0_l)
            ax.set_xlabel('x position (m)', rotation=150)
            ax.set_ylabel('y position (m)')
            ax.set_zlabel('z position (m)', rotation=60)
            plt.title('Time = ' + str(time_interval[ts]) + ' s')
            plot = ax.scatter(x[ts],y[ts],z[ts], c = pe_array[ts].ravel(),cmap = plt.cm.hot)
            #the add axes gives parameters to adjust [left, bottom, width, height]
            norm = colors.Normalize(vmin=0.,vmax=1.2*np.max(pe_array))
            mappable = cm.ScalarMappable(norm=norm, cmap = plt.cm.hot)
            mappable.set_array(plot)
            cb = fig.colorbar(mappable, cax = fig.add_axes([0.1, 0.4, 0.01, 0.3]))


        writer = animation.FFMpegWriter(fps = fps)
        ani = animation.FuncAnimation(fig, update, frames, fargs = (x_plot, y_plot, z_plot, total_energy))

        ani.save(folder_dir + 'mss_timeplot_' + date + '.mp4', writer= writer)

    ###################################################################################################
    # track waves and write velocity to text file#
    ###################################################################################################

    average_wave_velocity = {}
    average_wave_velocity["highval"] = high_val
    average_wave_velocity["lowval"] = low_val
    average_wave_velocity["numlayers"] = num_layers
    average_wave_velocity["average k value"] = k_avg_val



    x = np.linspace(0,n-2)
    y = np.linspace(0,n-2)
    tracked_position_list = []
    for xi in x:
        for yi in y:
            tracked_position_list.append([-1,xi,yi])

    last_layer_velocities = wave_tracker1(start_position, tracked_position_list, pe_array, time_interval, Pos_x, Pos_y, Pos_z)
    last_layer_velocities_list = open(folder_dir+"last_layer_velocities_" + date + str(file_index) + ".txt", "w+" )
    last_layer_velocities_list.write(str(last_layer_velocities))

    # write average velocity, standard deviation of velocities
    average_velocity = np.mean(last_layer_velocities)
    average_wave_velocity["average velocity"] = average_velocity
    average_wave_velocity["standard deviation velocity"] = np.std(last_layer_velocities)

    # plot histogram
    plt.figure()
    plt.text(0, 1, str(average_velocity))
    plt.hist(last_layer_velocities, int((n*n)/4))
    plt.xlabel('wave velocity (m/s)')
    plt.ylabel('frequency')

    plt.savefig(folder_dir + '_velocity_hist_'+ date + '.png')

    average_wave_velocity_list = open(folder_dir+"wave_cross_section_info_" + date + str(file_index) + ".txt", "w+" )
    average_wave_velocity_list.write(str(average_wave_velocity))

    ###################################################################################################
    # track last-layer mass with lowest distance from start position #
    ###################################################################################################
    middle = -1,int(n/2-1.5),int(n/2-1.5)
    middle_potential = pe_array[:, middle[0], middle[1], middle[2]]
    middle_kinetic = ke_array[:, middle[0], middle[1], middle[2]]


    fig, (ax1, ax2) = plt.subplots(1,2)

    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('energy (J)')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('energy (J)')


    plt.subplots_adjust(wspace = 0.4, hspace = 0.4)

    ax1.plot(time_interval, middle_potential)
    ax2.plot(time_interval, middle_kinetic)


    ax1.set_title('kinetic energy of the middle mass in the receiving layer')
    ax2.set_title('potential energy of the middle mass in the receiving layer')



    plt.savefig(folder_dir + '_middle_energy_plot_'+ date + '.png')


    ###################################################################################################
    # Global energy plots #
    ###################################################################################################

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2,2)
    # create arrays
    ke_sum = np.zeros((len(time_interval)-1))
    pe_sum = np.zeros((len(time_interval)-1))
    ke_max = np.zeros((len(time_interval)-1))
    pe_max = np.zeros((len(time_interval)-1))
    for ts in range(len(time_interval)-1):
        ke_sum[ts] =  np.sum(ke_array[ts])
        pe_sum[ts] =  np.sum(pe_array[ts])
        ke_max[ts] =  np.max(ke_array[ts])
        pe_max[ts] =  np.max(pe_array[ts])

    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('energy (J)')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('energy (J)')
    ax3.set_xlabel('time (s)')
    ax3.set_ylabel('energy (J)')
    ax4.set_xlabel('time (s)')
    ax4.set_ylabel('energy (J)')

    plt.subplots_adjust(wspace = 0.4, hspace = 0.4)

    ax1.plot(time_interval[:-1], ke_sum)
    ax2.plot(time_interval[:-1], pe_sum)
    ax3.plot(time_interval[:-1], ke_max)
    ax4.plot(time_interval[:-1], pe_max)

    ax1.set_title('total kinetic energy')
    ax2.set_title('total potential energy')
    ax3.set_title('max kinetic energy ')
    ax4.set_title('max potential energy')


    plt.savefig(folder_dir + 'energy_plot_'+ date + '.png')


    ###################################################################################################
    # Plot the 1D projection of energy propagation in the z direction. #
    ###################################################################################################
    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    wave_prop_plot = ax1.matshow(np.transpose(total_energy_1d), aspect='auto', extent=[0, np.max(time_interval), 0, nz-2], origin='lower')
    plt.gca().xaxis.tick_bottom()  # switches x-axis from top to bottom
    plt.title("Energy(J)")
    plt.ylabel("Position (m)")
    plt.xlabel("Time (s)")
    # ax1.legend(['average velocity:'+ str() + 'm/s'])
    plt.colorbar(wave_prop_plot, label = '\u221A Energy (\u221AJ)')
    plt.savefig(folder_dir + '_waveprop_1d_t' + str(time_limit) + 's.png')

    ###################################################################################################
    # parameter and hyper parameter tables  #
    ###################################################################################################
    str1 = 'time interval: ' + str(time_limit) + ' s' + '\n'
    str2 = 'average stiffness = ' + str(np.mean(k)) + 'N/m' + '\n'
    str3 = 'average particle mass = ' + str(np.mean(M)) + 'kg' + '\n'
    str4 = 'total mass = ' + str(np.sum(M)) + 'kg' + '\n'
    str5 = 'mss dimensions = ' + str(n) + ' x ' + str(n) + ' x ' + str(nz) + 'particles' + '\n'
    str5 = 'linear damping constant = ' + str(damping_threshold) + ' * ' + str(damping_constant_critical) + ' = ' + str(damping_constant) + '\n'
    str6 = 'corner mass initial speed = ' + str(initial_velocity) + 'm/s'
    L = [str1, str2, str3, str4, str5, str6]

    parameter_list = open(folder_dir+"parameter_list_" + date + str(file_index) + ".txt", "w+" )
    for l in L:
        parameter_list.writelines(l)



if __name__ == "__main__":
    main()
