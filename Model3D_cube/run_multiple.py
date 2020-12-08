import numpy as np
import os

# Run multiple experiments!

eq_lengths = [1.0]
# num_layers_list = np.arange(11,12,1) #11 layers
num_layers_list = np.arange(1,2,1)
damping_constants = [2]
velocities = [5]
for v in velocities:
    for p in damping_constants:
        for e in eq_lengths:
            for l in num_layers_list:
                directory = '3D_Wave_Data/010220/_p' + str(p) + '_e' + str(e)
                if not os.path.exists(directory):
                    os.makedirs(directory)
                text = ('python3 run.py -d 010220 -t 10 -v ' + str(v) + ' -n 3 -z 10 -l ' + str(l) +  ' -i 1 -j 2 -p ' + \
                str(p) + ' -e ' + str(e) + ' ' + directory+'/_l' + str(l))
                os.system(text)

print('done')
