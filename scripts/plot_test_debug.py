#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import glob, os, sys
from tqdm import tqdm

os.chdir("../test/build")
for filename in tqdm(glob.glob("*.csv")):
  data = np.genfromtxt(filename, delimiter=',')
  title = filename.split(".")[0]
  t = data[:,0]
  qhat = data[:,2:6]
  q = data[:,6:10]
  euler = data[:,10:13]
  omega = data[:,13:16]
  omegahat = data[:,16:19]
  error = data[:,19]

  fig =  plt.figure()
  fig.canvas.set_window_title(title + "_quat")
  for i in range(4):
    ax = plt.subplot(4,1,i+1)
    plt.plot(t, qhat[:,i], label="est")
    plt.plot(t, q[:,i], label="truth")
    if i == 3:
      ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05),
          fancybox=True, shadow=True, ncol=2)
  plt.savefig(title + "_quat.png")  

 
  fig =  plt.figure()
  fig.canvas.set_window_title(title + "_omega")
  for i in range(3):
    plt.subplot(3,1,i+1)
    plt.plot(t, omegahat[:,i], label="est")
    plt.plot(t, omega[:,i], label="truth")
    if i == 3:
      ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05),
          fancybox=True, shadow=True, ncol=2)
  plt.savefig(title + "_omega.png")  

  fig = plt.figure()
  fig.canvas.set_window_title(title + "_error")
  plt.plot(t, error)
  plt.savefig(title + "_error.png")
  
  
