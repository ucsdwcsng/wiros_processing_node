#! /usr/bin/env python3
import rosbag
import sys
import csi_utils.constants as constants
import csi_utils.transform_utils as transform_utils
import csi_utils.pipeline_utils as pipeline_utils
from os.path import join
import os
import tqdm
import numpy as np
import argparse


parse = argparse.ArgumentParser()
parse.add_argument("bag", help="Input CSI bag file")
args,_ = parse.parse_known_args()
bag = rosbag.Bag(args.bag)

channels = {}
times = {}
aoas = {}
rssis = {}
profs = {}
aoa_sensors = {}
rx = None
out = os.getcwd()
for topic, msg, t in tqdm.tqdm(bag.read_messages('/csi')):
    csi = pipeline_utils.extract_csi(msg)
    #assuming this does not change
    rx = msg.rx_id
    mac = pipeline_utils.mac_to_str(tuple(msg.txmac))

    key = f"{mac}-{msg.chan}-{msg.bw}"

    if key not in channels.keys():
        channels[key] = []
        times[key] = []
        rssis[key] = []

    channels[key].append(csi)
    times[key].append(t.to_sec())
    rssis[key].append(msg.rssi)

maxk = ''
maxlen = 0
for key in channels:
	if len(channels[key]) > maxlen:
	    maxk = key
	    maxlen = len(channels[key])
if maxk == '':
	print("No CSI in bag file")
	exit(1)

ksplit = maxk.split("-")
print(f"Generating Compensation for {rx} with chanspec {ksplit[1]}/{ksplit[2]} from txmac {ksplit[0]}")
Hcomp = np.asarray(channels[maxk])[:,:,:,0].transpose(1,2,0)
#reduce to singular vector
nsub = Hcomp.shape[0]
nrx = Hcomp.shape[1]
Hcomp = Hcomp.reshape((-1,Hcomp.shape[2]))
_,vec=np.linalg.eig(Hcomp@Hcomp.conj().T)
Hcomp = vec[:,0].reshape((nsub,nrx,1))
Hcomp = np.exp(-1.0j*np.angle(Hcomp))
np.save(join(out,f"{rx}-{ksplit[1]}.npy"), Hcomp)
print(f"Created {join(out,f'{rx}-{ksplit[1]}.npy')}")
