import numpy as np
import rosbag
import scipy.io as sio
import sys
from csi_utils import pipeline_utils


file = "/path/to/bag-file"


bag = rosbag.Bag(file)
print("Loaded bag file; extracting to mat file now ...")

all_csi  = []

for topic, msg, t in bag.read_messages(topics=['/csi']):

   csi = pipeline_utils.extract_csi(msg)
   all_csi.append(csi[None])

all_csi = np.concatenate(all_csi, axis=0)
sio.savemat(f"{file.split('.')[0]}.mat",{"csi": all_csi})


bag.close()
print(f"Bag file extracted to {file.split('.')[0]}.mat")
