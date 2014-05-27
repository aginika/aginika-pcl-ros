#! /usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import numpy as np
import time
import glob

plt.ion()
plt.rc('font', **{'family': 'serif'})
fig = plt.figure()
counter = -1

import sys

argvs = sys.argv
argc = len(argvs)
directory = argvs[1]

for f in glob.glob( directory + "*.txt"):
    for l in open(f).readlines():
        counter += 1

        double_strings = l.split(",");

        print double_strings
        data = map(lambda x: float(x), double_strings[:len(double_strings)-1])

        data = np.array(data)

        ax = fig.add_subplot(111)
        ax.set_title('Histogram'+str(counter), size=16)
        ax.set_xlabel('Score', size=14)
        ax.set_ylabel('Frequency', size=14)

        ax.bar( range(0, len(data)), data)
        ax.grid(True)
        fig.canvas.draw()
        time.sleep(1e-7)
        fig.clf()
