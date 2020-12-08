import numpy as np
import os

# Run multiple experiments!

eq_lengths = [1.0]
# num_layers_list = np.arange(11,12,1) #11 layers
num_layers_list = np.arange(1,2,1)
damping_constants = [1]
velocities = [5] # acceleration
time = [5]
i = 1
for t in time:
    for v in velocities:
        for p in damping_constants:
            for e in eq_lengths:
                for l in num_layers_list:
                    directory = '3D_Wave_Data/010620/_i' + str(i) +  '_p' + str(p) + '_e' + str(e) + '_l' + str(l) + '_t' + str(t)
                    if not os.path.exists(directory):
                        os.makedirs(directory)
                    text = ('python3 run.py -d 010620 -t '+  str(t) + ' -v ' + str(v) + ' -n 5 -z 51 -l ' + str(l) +  ' -i ' +  str(i)+ ' -j 2 -p ' + \
                    str(p) + ' -e ' + str(e) + ' ' + directory+'/_l' + str(l))
                    os.system(text)

print('done')
