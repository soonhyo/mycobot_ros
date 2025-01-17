#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

# read from bag file
bag = rosbag.Bag(filename)
np_jointangles=[]
np_times=[]
for topic, msg, t in bag.read_messages():
    if topic=="/joint_states":
        np_jointangle=list(msg.position)
        np_time=[]
        
        np_time.append(t.secs)
        np_time.append(t.nsecs)
        
        np_jointangles.append(np_jointangle)
        np_times.append(np_time)
        
np_jointangles = np.array(np_jointangles, dtype='float64')
np_times = np.array(np_times, dtype='float64')

print(np_jointangles.shape)
bag.close()

# reform time
start_sec=np_times[0,0]
start_nsec=np_times[0,1]
t=np.zeros(np_times.shape[0],dtype='float64')
for i in range(np_times.shape[0]):
    t[i]=(np_times[i,0]-start_sec)+(np_times[i,1]-start_nsec)/1000000000.0

# plot    
fig, axes = plt.subplots(2, 3, figsize=(16,12))
c = 0
for i in range(2):
    for j in range(3):
        axes[i][j].plot(t, np_jointangles[:,c], 'r', label="j"+str(c+1))
        axes[i][j].set_title("j"+str(c+1))
        axes[i][j].set_xlabel("time[s]")
        axes[i][j].set_ylabel("angle[rad]")
        c += 1
plt.show()

