import numpy as np

############################################################################
# Randomization function #
############################################################################
def randomize_k(k, k_lowerbound, k_upperbound):
    k = k.flatten()
    for k_val in range(len(k)):
        k[k_val] = np.random.uniform(k_lowerbound, k_upperbound)

############################################################################
# Randomization function #
############################################################################
def z_layers(k, num_layers, low_val, high_val):
    # take number of layers, generate a num list of that length
    avg_val = high_val - (high_val-low_val)/2
    list = np.random.uniform(low_val, high_val, num_layers)
    list_avg_val = np.mean(list)
    while list_avg_val > avg_val:
        list = np.random.uniform(low_val, high_val, num_layers)
        list_avg_val = np.mean(list)
    mean_difference = avg_val - list_avg_val
    list += mean_difference
    k_avg_val = np.mean(list)

    increment, modulus = divmod(np.shape(k)[1],num_layers)

    counter = 0
    for i in range(num_layers):
        if i == num_layers-1:
            k[:,counter:counter+modulus ,:,:] = list[i]
        else:
            k[:,counter:counter+increment ,:,:] = list[i]
            counter+= increment


    return k, k_avg_val
