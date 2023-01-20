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

#create .npz file for python
export_npz = True
#create .mat file for matlab
export_mat = True

#path to a bag file
bag = rosbag.Bag("~/raw_data.bag")

#dir to output processed data
out = "~/data/"

#optional compensation path, set to None to not use.
comp = "~/compensation/192.168.1.1-42.npy"

#AoA algorithm (see README.md)
algo = "aoa_only"

#Save the profile images
save_prof = True


#smooth over last n packets according to algo
smoothing_window = 40
#aoa search space
theta_space = np.linspace(-np.pi,np.pi,360)
#tof search space
tof_space = np.linspace(-40,40,240)

#antenna position (interleaved x-y), see antennas.md
rx_pos = np.asarray([0,  0,
    0,  0.019,
    -0.019,  0.0,
    -0.019,  0.019]).reshape((-1,2))


if comp is not None:
    comp = np.load(comp)
else:
    comp = None
    print("warning: no comp file provided")

if algo is not None:
    use_aoa = True
    aoa_algo = algo
else:
    use_aoa = False

channels = {}
times = {}
aoas = {}
rssis = {}
profs = {}
aoa_sensors = {}
rx = None

if not export_mat and not_export_npz:
    print("You have not set export_mat or export_npz, exiting.")
    exit(0)

for topic, msg, t in tqdm.tqdm(bag.read_messages('/csi')):
    csi = pipeline_utils.extract_csi(msg, comp)
    #assuming this does not change
    rx = msg.rx_id
    mac = pipeline_utils.mac_to_str(tuple(msg.txmac))

    key = f"{mac}-{msg.chan}-{msg.bw}"

    if key not in channels.keys():
        channels[key] = []
        times[key] = []
        rssis[key] = []

        if use_aoa:

            if aoa_algo == 'full_svd':
                aoa_sensors[key] = transform_utils.full_svd_aoa_sensor(rx_pos, theta_space, tof_space, smoothing_window)
            elif aoa_algo == 'rx_svd':
                aoa_sensors[key] = transform_utils.rx_svd_aoa_sensor(rx_pos, theta_space, tof_space, smoothing_window)
            elif aoa_algo == 'aoa_only':
                aoa_sensors[key] = transform_utils.aoa_sensor_1d(rx_pos, theta_space, smoothing_window)
            elif aoa_algo == 'fft':
                aoa_sensors[key] = transform_utils.fft_aoa_sensor(rx_pos, theta_space, tof_space)
            elif aoa_algo == 'music':
                aoa_sensors[key] = transform_utils.full_music_aoa_sensor(rx_pos, theta_space, tof_space, smoothing_window)
            elif aoa_algo == 'aoa_music':
                aoa_sensors[key] = transform_utils.music_aoa_sensor_1d(rx_pos, theta_space, smoothing_window)
            else:
                print(f"Invalid AoA {aoa_algo}")

            if save_prof:  
                print(list(aoa_sensors.keys()))
                aoa_sensors[key].return_profile = True

            aoas[key] = []
            profs[key] = []

    channels[key].append(csi)
    times[key].append(t.to_sec())
    rssis[key].append(msg.rssi)

    if use_aoa:
        res = aoa_sensors[key](csi,(msg.chan,msg.bw*1e6))
        aoas[key].append(res[0])
        profs[key].append(pipeline_utils.compress_profile(res[1]))

if export_mat:
    import scipy.io as sio

for key in channels:

    f_dict = {'H':np.asarray(channels[key]),
              't':np.asarray(times[key]),
              'rssi':np.asarray(rssis[key]),
              'rx':np.asarray(rx)
    }
    
    if use_aoa:
        f_dict['aoa'] = np.asarray(aoas[key])
        if save_prof:
            f_dict['prof'] = np.asarray(profs[key])
    
    if export_npz:        
        np.savez(join(out, f"{key}.npz"), **f_dict)
        print(f"Created {join(out, f'{key}.npz')}")
    if export_mat:
        sio.savemat(join(out, f"{key}.mat"), f_dict)
        print(f"Created {join(out, f'{key}.mat')}")
                