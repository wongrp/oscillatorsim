import numpy as np
import os

# Run multiple experiments!

eq_lengths = [0.1]
# num_layers_list = np.arange(11,12,1) #11 layers
num_layers_list = np.arange(1,2,1)
damping_constants = [0]
velocities = [10] # acceleration
time = [25]
i = 1
n = 5
nz = 11

for t in time:
    for v in velocities:
        for p in damping_constants:
            for e in eq_lengths:
                for l in num_layers_list:
                    directory = '3D_Wave_Data/012420/_i' + str(i) +  '_p' + str(p) + '_e' + str(e) + '_l' + str(l) + '_t' + str(t) + '_'+ str(n) + 'x' + str(n) + 'x' + str(nz)
                    if not os.path.exists(directory):
                        os.makedirs(directory)
                    text = ('python3 run.py -d 012420 -t '+  str(t) + ' -v ' + str(v) + ' -n ' + str(n) + ' -z ' + str(nz) + ' -l ' + str(l) +  ' -i ' +  str(i)+ ' -j 2 -p ' + \
                    str(p) + ' -e ' + str(e) + ' ' + directory+'/_')
                    os.system(text)

print('done')
