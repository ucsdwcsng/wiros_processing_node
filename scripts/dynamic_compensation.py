# -*- coding: utf-8 -*-
import numpy as np

import csi_utils.transform_utils as transforms
import csi_utils.constants as constants
import csi_utils.comp_utils as comp_utils
import rosbag

import matplotlib.pyplot as plt
import tqdm
from os.path import join

#output directory
data_dir = "~/data/results/"

#channel files from generate_channel_files.py
Hfile = np.load("~/data/11:11:11:11:11:11-155-80.npz")
#desired chanspec
chan = 155
bw = 80e6
#ground AoA (time-synced to channels)
g_aoa = np.load("~/data/ground_aoa.npy")
#receiver antenna array, see antennas.md
rx = np.asarray([0,  0,
    0,  0.019,
    -0.019,  0.0,
    -0.019,  0.019]).reshape((4,2))

freqs = constants.get_channel_frequencies(chan,bw)

#load channels
H = Hfile['H']


#if you have a large number of channels you may want to add code to subsample for memory usage here

#remove zero-channels
nz_idx = np.all(np.all(np.all(H != 0, axis=1) != 0, axis=1), axis=1)
H = H[nz_idx]
g_aoa = g_aoa[nz_idx]

H = H.transpose(0,2,1,3)
H = H.reshape((H.shape[0], -1,H.shape[3]))

nsample = g_aoa.shape[0]
#%%

#generate steering vectors
S = transforms.aoa_steering_vec(rx, freqs, g_aoa).transpose(0,2,1).reshape((g_aoa.shape[0],-1)).conj()

print("Extracting channels...")
for pkt in range(H.shape[0]):
    #remove phase of steering vectors from channels
    H[pkt,:,:] = np.diag(S[pkt,:]) @ H[pkt,:,:]

#generate noise space
H = H.transpose((1,2,0))
H = H.reshape((H.shape[0],-1))
u,s,_ = np.linalg.svd(H)
s /= np.max(s)
import pdb
pdb.set_trace()
u = u[:,1:]


#want to minimize projection onto noise space, pre-compute M for speed
M = u @ u.conj().T

# '''
#To ensure we get the right compensation value, we perform a broad search across the initialization space(in receiver antennas only,) and do levenberg-marquardt to optimize from there, as well as optimizing for the per-subcarrier calibration values.
ru_4 = [0.0, np.pi/2, np.pi, 3*np.pi/2]
cs = []
ls = []
pbar = tqdm.trange(len(ru_4)**3)
for tx1 in range(4):
    for tx2 in range(4):
        for tx3 in range(4):
            initc = np.kron(np.asarray([1.0,ru_4[tx1],ru_4[tx2],ru_4[tx3]])[:,np.newaxis],np.ones((freqs.shape[0],1)))
            c, l = comp_utils.music_compensation(H,g_aoa,initc,rx,freqs,return_loss=True,verbose=True, M=M)
            c = c.conj()
            c = c.reshape((1,4,freqs.shape[0])).T
            cs.append(c)
            ls.append(l)
            pbar.update(1)
np.save(join(data_dir, "comp-losses.npy"), np.asarray(ls))
np.save(join(data_dir, "result-comps.npy"), np.asarray(cs))

ls = np.load(join(data_dir, "comp-losses.npy"))
cs = np.load(join(data_dir, "result-comps.npy"))
# '''

#optional- test a specific initialization
# '''
# initc = np.angle(compold.conj().T.reshape((-1,1)))
# initc = np.ones((208,1))
# initc = np.random.uniform(0,2*np.pi,(936,1))

# c, l = comp_utils.music_compensation(H,g_aoa,initc,rx,freqs,return_loss=True,verbose=True,M=M)
# c = c.conj()
# c = c.reshape((1,4,52)).T
# '''

sort_idx = np.argsort(ls)
ls = ls[sort_idx]
cs = cs[sort_idx,:,:,:]

#%%
#pick compensation that had the lowest loss
c = cs[0,:,:,:]

theta_vals = np.linspace(-np.pi/2,np.pi/2,360)
d_vals = np.linspace(-40,40,1000)
Theta_p, Tau_p = transforms.fft_mat(rx, freqs, theta_vals, d_vals)
comp_prof = Theta_p @ c[:,:,0].conj().T @ Tau_p
fig = plt.figure(figsize=(16,16))

fig.set_tight_layout(True)
ax = fig.add_subplot()
ax.imshow(np.abs(comp_prof), origin='lower', extent=(d_vals[0], d_vals[-1], theta_vals[0], theta_vals[-1]), zorder=-2, cmap='jet')
ax.set_aspect((d_vals[-1]-d_vals[0])/(theta_vals[-1]-theta_vals[0]))

ax.axis('off')
ax.set_title("Calculated Compensation")

c = np.exp(1.0j*np.angle(c))
#%%
np.save(join(data_dir, "computed_comp.npy"), c)
plt.show()

