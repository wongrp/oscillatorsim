
import numpy as np


############################################################################
# Distance Calculator #
############################################################################

def distance_calculator(particle, adjacent_particle):
    xyz_distance = adjacent_particle - particle
    return xyz_distance

###################################################################################################
# Angle Calculator #
###################################################################################################
#x, y, z components of the distance to a particular mass, be it xp, xn, etc...,
# Calculates the angle between the two masses with 0 deg being the angle at equilibrium (in this case a cube of masses)
# Formula: arctan(z component / (square root of (x component^2 + y component^2)))

# need to write a for loop that gets rid of all the ones that are undefined and set them to pi/2

def angle_calculator(xyz_distance):
    x_component = xyz_distance[0]
    y_component = xyz_distance[1]
    z_component = xyz_distance[2]

    # xy plane projection length
    xy_projection = np.power(np.power(x_component,2)+np.power(y_component,2),0.5)
    # arg1 = np.argwhere(x_component == 0)
    # arg2 = np.argwhere(xy_projection == 0)

    # adjust
    for i in range(len(x_component)):
        for j in range(len(x_component[i])):
            for k in range(len(x_component[i,j])):
                if x_component[i,j,k] == 0:
                    x_component[i,j,k] += 1e-12
                if xy_projection[i,j,k] == 0:
                    xy_projection[i,j,k] += 1e-12

    # 3D angle; angle between xy plane and z axis
    angle_xy_z = np.arctan(abs(z_component/xy_projection))
    angle_xy = np.arctan(abs((y_component)/(x_component))) # 2D angle; angle between x and y projection


    return angle_xy_z, angle_xy

############################################################################
# length deviation calculator #
############################################################################
def length_deviation_calculator(xyz_distance, L0):
    x_dist = xyz_distance[0]
    y_dist = xyz_distance[1]
    z_dist = xyz_distance[2]
    length_deviation = np.power((np.power(x_dist,2)+np.power(y_dist,2)+np.power(z_dist,2)),0.5)-L0
    return length_deviation

############################################################################
# hooke's force solver  #
############################################################################
def hookes_force_magnitude_solver(particle, adjacent_particle, L0, k_array):
    xyz_distance = distance_calculator(particle, adjacent_particle)
    length_deviation = length_deviation_calculator(xyz_distance, L0)
    hookes_force_magnitude = k_array*length_deviation
    return hookes_force_magnitude

def hookes_force_component_solver(hookes_force_magnitude, angle_xy, angle_xy_z):
    x_force = hookes_force_magnitude*np.cos(angle_xy_z)*np.cos(angle_xy)
    y_force = hookes_force_magnitude*np.cos(angle_xy_z)*np.sin(angle_xy)
    z_force = hookes_force_magnitude*np.sin(angle_xy_z)
    return x_force, y_force, z_force


def hookes_force_solver(particle, adjacent_particle, L0, k_array):
    xyz_distance = distance_calculator(particle, adjacent_particle)
    angle_xy, angle_xy_z = angle_calculator(xyz_distance)
    length_deviation = length_deviation_calculator(xyz_distance, L0)
    hookes_force_magnitude = hookes_force_magnitude_solver(particle, adjacent_particle, L0, k_array)
    x_force, y_force, z_force = hookes_force_component_solver(hookes_force_magnitude, angle_xy, angle_xy_z)

    # sign correction
    for n in range(len(xyz_distance)):
        for i in range(len(xyz_distance[n])):
            for j in range(len(xyz_distance[n,i])):
                for k in range(len(xyz_distance[n,i,j])):
                    if xyz_distance[n, i,j,k] < 0:
                        if n == 0: # that means x component < 0
                            x_force = - x_force
                        if n == 1: # that means y component < 0
                            y_force = - y_force
                        if n == 2:# that means z component < 0
                            z_force = - z_force
    return length_deviation, x_force, y_force, z_force
