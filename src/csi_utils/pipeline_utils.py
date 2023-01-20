import numpy as np
import csi_utils.constants as constants
import csi_utils.transform_utils as transform_utils

NUM_TOTAL_TX = 4
NUM_TOTAL_RX = 4
BW2NUM_SC = 3.2e-6

def nts(H, freq):
   #H: n_sub x n_rx x n_measurements
   for tx in range(H.shape[2]):
      nTs = H.shape[1]
      hpk = H[:, :, tx]
      line = np.polyfit(freq, np.unwrap(np.angle(hpk), axis=0), 1)
      tch = np.min(line[0,:])
      subc_angle = np.exp(-1.0j*tch*freq)
      H[:,:, tx] = hpk*subc_angle[:,np.newaxis]
   return H


def extract_csi(msg, comp=None, apply_nts=True, valid_tx=None):
   bw = msg.bw*1e6
   csi_r = np.asarray(msg.csi_real)
   csi_i = np.asarray(msg.csi_imag)
   csi = csi_r + 1.0j*csi_i
   csi = csi.reshape((NUM_TOTAL_TX, NUM_TOTAL_RX, int(bw*BW2NUM_SC))).T
   if valid_tx is not None:
      csi = csi[:,:,valid_tx]

   #these subcarriers are rotated. need to find justification for why this is the case
   if bw == 80e6:
      csi[:64]*= -1
   if bw == 40e6:
      csi[:64]*= -1j

   csi = csi[constants.subcarrier_indices[bw]]

   if bw == 80e6:
      csi[117] = csi[118]


   if apply_nts:
      csi = nts(csi, constants.subcarrier_frequencies[bw])

   if comp is not None:
      csi *= comp

   return csi

def compress_profile(prof):
   return (255*prof/np.max(prof)).astype(np.uint8)

def mac_to_str(mactup):
   assert len(mactup) == 6, f"Invalid MAC tuple: {mactup}"
   return "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}".format(mactup[0], mactup[1], mactup[2], mactup[3], mactup[4], mactup[5])
   
