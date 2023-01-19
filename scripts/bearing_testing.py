#! /usr/bin/env python3
import rosbag
import sys
import  constants
import transform_utils
import pipeline_utils
import tqdm
import numpy as np
from matplotlib import pyplot as plt

print("Loading Bag file ...")
# bag = rosbag.Bag("/media/aarun/wd_backup/p2slam_realworld/viwid_bags/7-22-short_w_paths.bag")
bag = rosbag.Bag("/home/aarun/Research/data/ros_csi_testing/test3_20mhz_ch36_224.bag")
comp = np.load("/home/aarun/Research/data/ros_csi_testing/compensations/comp_224_corrected.npy")
# comp = np.load("/home/aarun/Research/data/asus_calib/192.168.43.221-42.npy")
# comp = None
use_aoa = True
aoa_algo = "full_svd" # "fft", "full_svd", "rx_svd", "aoa_only", "spotfi", "music", "aoa_music"
save_prof = False
print("... Done")

#set comp for four tx antennas form the first tx antenna;
comp = np.repeat(comp[:, :, 0:1], 4, axis=2)

#%% Extract Stored AoA
aoas_stored = {}
times_stored = {}
for ii, (topic, msg, t) in tqdm.tqdm(enumerate(bag.read_messages('/aoa'))):
   key = pipeline_utils.mac_to_str(tuple(msg.txmac))

   if not key in aoas_stored.keys():
      aoas_stored[key] = []
      times_stored[key] = []

   aoas_stored[key].append(msg.aoa)
   times_stored[key].append(t.to_sec())

#%% Compute AoA for debugging

# thetas = np.linspace(-np.pi, np.pi, 180)
thetas = np.linspace(-np.pi/2, np.pi/2, 180)
tofs = np.linspace(-40, 40, 180)


channels = {}
times = {}
aoas = {}
rssis = {}
profs = {}
aoa_sensors = {}

# Square array
# sqside = 0.00945 * 2
# rxpos = np.asarray([[0, 0], [0, sqside], [-sqside, 0], [-sqside, sqside]])

# Linear array
ant_sep = 0.026
rxpos = np.asarray([[0, -ii*ant_sep] for ii in range(4)])

for ii, (topic, msg, t) in tqdm.tqdm(enumerate(bag.read_messages('csi'))):

   csi = pipeline_utils.extract_csi(msg, comp)
   key = pipeline_utils.mac_to_str(tuple(msg.txmac))

   if key not in channels.keys():
      channels[key] = []
      times[key] = []
      rssis[key] = []
      if use_aoa:
         if aoa_algo == 'full_svd':
            aoa_sensors[key] = transform_utils.full_svd_aoa_sensor(rxpos, [0], thetas, tofs, 40)
         elif aoa_algo == 'rx_svd':
            aoa_sensors[key] = transform_utils.rx_svd_aoa_sensor(rxpos, thetas, tofs, 40)
         elif aoa_algo == 'aoa_only':
            aoa_sensors[key] = transform_utils.aoa_sensor_1d(rxpos, thetas, 40)
         elif aoa_algo == 'fft':
            aoa_sensors[key] = transform_utils.fft_aoa_sensor(rxpos, [0], thetas, tofs)
         elif aoa_algo == 'music':
            aoa_sensors[key] = transform_utils.full_music_aoa_sensor(rxpos, thetas, tofs, 40)
         elif aoa_algo == 'aoa_music':
            aoa_sensors[key] = transform_utils.music_aoa_sensor_1d(rxpos, thetas, 40)
         elif aoa_algo == 'spotfi':
            aoa_sensors[key] = transform_utils.spotfi_sensor(rxpos, thetas, tofs)

         if save_prof:
            aoa_sensors[key].return_profile = True

         aoas[key] = []
         profs[key] = []

   channels[key].append(csi)
   times[key].append(t.to_sec())
   rssis[key].append(msg.rssi)

   # if (aoa_algo == 'spotfi' or aoa_algo == 'music') and ii % 10 != 0:
   #    aoas[key].append(np.nan)
   #    continue

   if use_aoa:
      res = aoa_sensors[key](csi, (msg.chan, msg.bw * 1e6))
      aoas[key].append(res[0])
      profs[key].append(res[1])

#%% Plot the AoA's
fig = plt.figure()
axs = fig.subplots(len(times.keys()), 1)
if (len(times.keys()) == 1):
   axs = [axs]

for ii, key in enumerate(times.keys()):

   cur_times = np.array(times[key]) - times[key][0]
   axs[ii].plot(cur_times[:len(aoas[key])], np.array(aoas[key])*180/np.pi, "o", label="computed AoA's")

   # cur_times_stored = np.array(times_stored[key]) - times_stored[key][0]
   # axs[ii].plot(cur_times_stored, np.array(aoas_stored[key])*180/np.pi, "o", label="stored AoA's")

plt.legend()
