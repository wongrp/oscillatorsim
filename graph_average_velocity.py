import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib import colors, cm
import os, getopt, sys

def usage():
    script = os.path.basename(__file__)
    print("\n\nUsage:  " + script + " [options] <output directory>")
    print('''
                    Options:
                    -h --help (help)
                    -x --xaxis
                    -y --yaxis
                    -e --errorbar (standard deviation)
                    <file name directory> (e.g. 3D_Wave_Data/avg_velocity_directory.txt)
                    <output file directory>
                    ''')
    sys.exit()

def main():
    opts, files = getopt.getopt(sys.argv[1:], "h:x:y:e:", [ "help", "xaxis", "yaxis", "errorbar"])

    if len(files) != 2:
        usage()

    # defaults:
    parameters = {}
    parameters['x'] = 'numlayers'
    parameters['y'] = 'average velocity'
    parameters['e'] = 'standard deviation velocity'
    parameters['u'] = 'number of layers'
    parameters['v'] = 'average velocity (m/s)'

    # loop over options:
    for option, argument in opts:
        if option in ("-h", "--help"):
            usage()
        elif option in ("-x", "--xaxis"):
            parameters['x'] = argument
        elif option in ("-y", "--yaxis"):
            parameters['y'] = argument
        elif option in ("-e", "--errorbar"):
            parameters['e'] = argument
        elif option in ("-u", "--xtitle"):
            parameters['u'] = argument
        elif option in ("-v", "--ytitle"):
            parameters['v'] = argument

    base = os.path.basename(files[0])
    input_video_name, input_video_extension = os.path.splitext(base)


    # parameters
    xaxis = parameters['x']
    yaxis = parameters['y']
    errorbar = parameters['e']
    xtitle= parameters['u']
    ytitle = parameters['v']

    # set file paths
    folder_dir = files[0]
    output_dir = files[1]

    file = open(folder_dir, 'r')
    read_file = file.read().splitlines()
    x = []
    y = []
    errorbars = []

    for directory in read_file:
        dict = eval(open(directory, 'r').read())
        xi = np.float(dict[xaxis])
        yi = np.float(dict[yaxis])
        error = np.float(dict[errorbar])
        x.append(xi)
        y.append(yi)
        errorbars.append(error)

    fig, ax = plt.subplots()
    ax.errorbar(x, y, yerr = errorbars, fmt = 'o')
    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_ylim(0,3*np.max(y))
    plt.grid()
    plt.savefig(output_dir)


if __name__ == "__main__":
    main()
