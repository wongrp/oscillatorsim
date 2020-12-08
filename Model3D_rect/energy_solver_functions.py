import numpy as np
################################################################################
# kinetic and potential energy calculators #
################################################################################
def calculate_kinetic_energy(M, V_x, V_y, V_z):
    v_term = np.power(V_x, 2) + np.power(V_y, 2) + np.power(V_z, 2)
    m_term = M[1:-1, 1:-1, 1:-1]
    ke = 0.5 * np.multiply(m_term, v_term)

    return ke

# the potential energy of each spring is split evenly between the two masses.
# Hence the /4. (0.5kx^2)/2 = (kx^2)/4
def calculate_potential_energy(k, dL_list):
    pe_list = []
    for i in range(len(k)):
        pe = (np.multiply(k[i],np.power(dL_list[i],2)))/4
        pe_list.append(pe)
    pe_total = np.sum(pe_list, axis = 0) # keep interior matrix structure
  
    return pe_total
