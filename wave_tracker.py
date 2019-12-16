import numpy as np


################################################################################
# track principle #
################################################################################
# Assuming the corner mass is perturbed:
# Corner mass -1,-1,-1 is perturbed. Track wave velocity of the 7 other corner masses
# [0,0,0] [0,0,-1] [0,-1,0] [0,-1,-1] [-1,0,0] [-1,-1,0] [-1,0,-1] [-1,-1,0]


def wave_tracker(start_position, tracked_position_list, pe_array, time_interval, Pos_x, Pos_y, Pos_z):
    average_wave_velocity = {}
    for t in range(len(pe_array)):
        for position in tracked_position_list:
            position_value = tracked_position_list[position]
            if pe_array[t, position_value[0],position_value[1],position_value[2]] > \
            0.05*np.mean(pe_array[:,start_position[0], start_position[1], start_position[2]] ) and \
            position not in average_wave_velocity:
                print(time_interval[t])
                print(pe_array[t, position_value[0],position_value[1],position_value[2]])
                distance = np.sqrt((Pos_x[t, start_position[0], start_position[1], start_position[2]]- \
                Pos_x[t,position_value[0],position_value[1],position_value[2]])**2 + \
                (Pos_y[t,start_position[0], start_position[1], start_position[2]]-Pos_y[t,position_value[0],position_value[1],position_value[2]])**2 + \
                (Pos_z[t,start_position[0], start_position[1], start_position[2]]-Pos_z[t,position_value[0],position_value[1],position_value[2]])**2)
                average_wave_velocity[str(position)] = distance / time_interval[t]

    return average_wave_velocity


def wave_tracker1(start_position, tracked_position_list, pe_array, time_interval, Pos_x, Pos_y, Pos_z):
    average_wave_velocity = []
    for i in range(len(pe_array[0, 0])):
        for j in range(len(pe_array[0, 0, i])):
            t = np.argmax(pe_array[:, -1,i,j])
            distance = np.sqrt((Pos_x[t, start_position[0], start_position[1], start_position[2]]- \
            Pos_x[t,-1,i,j])**2 + \
            (Pos_y[t,start_position[0], start_position[1], start_position[2]]-Pos_y[t,-1, i,j])**2 + \
            (Pos_z[t,start_position[0], start_position[1], start_position[2]]-Pos_z[t,-1, i,j])**2)
            average_wave_velocity.append(distance / time_interval[t])

    return average_wave_velocity
