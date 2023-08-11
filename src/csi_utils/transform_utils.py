import numpy as np
import csi_utils.constants as constants
eps = np.finfo(float).eps
import matplotlib.pyplot as plt

C_SPEED = 3e8
SPOTFI_THRESH = 0.1
EPS = np.finfo('float').eps

def fft_mat(rx, freqs, theta, d):
    '''
    rx: AP antenna positions, n_rx x 2, each antenna should be x,y or theta,r.
    freqs: subcarrier frequencies
    theta: theta search space
    d: distance search space

    returns: Theta, Tau steering matrices, pre conjugated.
    so you can compute a profile as such: prof = Theta @ H.T @ Tau
    '''
    #wave number
    k = 2*np.pi*np.mean(freqs)/(C_SPEED)

    #steering matrix for AoA
    theta = np.repeat((theta[:,np.newaxis]), rx.shape[0], 1)

    #compute relative phase offset from origin and turn to exponential
    Theta = np.exp(-1.0j*k*(rx[:,0]*np.cos(theta) + rx[:,1]*np.sin(theta)))

    #steering matrix for ToF
    omega=2*np.pi*freqs[:,np.newaxis]/(C_SPEED)
    d = d[np.newaxis,:]
    Tau = np.exp(-1.0j*omega*d)

    return Theta, Tau

def argmaxlocal(im, thresh=0.0, exclude_borders=True):
   if im.shape[1] > 1:
       #return np.fliplr(np.flipud(feat.peak_local_max(im.T, indices=True, exclude_border=exclude_borders,
       #                                               min_distance=5, threshold_rel=thresh)))
       return np.asarray(np.unravel_index(np.argmax(im, axis=None), im.shape))[:,np.newaxis]
   else:
       return np.asarray([[np.argmax(im), 0]])

class full_svd_aoa_sensor:
    def __init__(self, rx_pos, theta_space, tof_space, pkt_window=40, valid_tx_ant=None):
        self.thetas = {}
        self.taus = {}
        self.rx_pos = rx_pos
        self.theta_space = theta_space
        self.tof_space = tof_space
        self.pkt_window = pkt_window
        self.chanspec_seen = set()
        self.Theta = {}
        self.Tau = {}
        self.svd_window = {}
        self.svd_roll = {}
        self.valid_tx_ant = valid_tx_ant
        self.prof_dim=2

    def __call__(self, H, chanspec):
        '''
        H: np.ndarray, N_subcarriers x N_receivers x N_transmitters
        chanspec: tuple, (channel,bandwidth)
        '''
        #setup the averaging window and compute DFT matrices
        bw = chanspec[1]
        if chanspec not in self.chanspec_seen:

            #load dft matrix
            self.Theta[chanspec], self.Tau[chanspec] = fft_mat(
                self.rx_pos,
                constants.get_channel_frequencies(chanspec[0],chanspec[1]),
                self.theta_space,
                self.tof_space
            )
            self.chanspec_seen.add(chanspec)

            # matrix of measurements. We want AoA/ToF,
            # so each measurement vector contains [num_receiver * num_subcarriers ]
            # the size will be [num_receivers*num_subcarriers x pkt_window]
            # we will average over the last pkt_window/T packets, given T transmit antennas.
            self.svd_window[chanspec] = np.zeros((self.rx_pos.shape[0]*constants.n_sub[bw],self.pkt_window),
                                                 dtype=np.complex128)

            # index of the oldest measurement, so we can overwrite it in the matrix
            self.svd_roll[chanspec] = 0

        # number of measurements to add to the matrix
        if self.valid_tx_ant is None:
            valid_tx_ant = range(H.shape[2])
        else:
            valid_tx_ant = self.valid_tx_ant

        # iterate across the new measurements
        c_roll = self.svd_roll[chanspec]
        for n in valid_tx_ant:
            #flatten the measurements to a vector and add to appropriate location in matrix
            self.svd_window[chanspec][:,c_roll] = H[:,:,n].reshape((-1),order='F')

            c_roll += 1
            if c_roll >= self.pkt_window:
                c_roll = 0
        self.svd_roll[chanspec] = c_roll

        # Take the first vector of the U matrix. I beleive this is faster than np.eig(x*x^H)
        # In the case where you have more measurements than
        # elements in the measurement vector then you should use np.eig instead, as computation of
        # the V matrix will be very expensive
        U_csi = np.linalg.svd(self.svd_window[chanspec])[0][:,0].reshape((constants.n_sub[bw],self.rx_pos.shape[0]), order='F').T
        #reshape the largest eigenvector to n_receivers x n_subcarriers

        #take 2d-fft
        prof = np.abs(self.Theta[chanspec] @ (U_csi @ self.Tau[chanspec]))
        #find max peak
        return self.theta_space[argmaxlocal(prof, thresh=0.99, exclude_borders=False)[0,0]], prof

class full_music_aoa_sensor:
    def __init__(self, rx_pos, theta_space, tof_space, pkt_window=40):
        self.thetas = {}
        self.taus = {}
        self.rx_pos = rx_pos
        self.theta_space = theta_space
        self.tof_space = tof_space
        self.pkt_window = pkt_window
        self.chanspec_seen = set()
        self.Theta = {}
        self.Tau = {}
        self.svd_window = {}
        self.svd_roll = {}
        self.prof_dim=2

    def __call__(self, H, chanspec):
        bw = chanspec[1]
        if chanspec not in self.chanspec_seen:
            self.Theta[chanspec], self.Tau[chanspec] = fft_mat(
                self.rx_pos,
                constants.get_channel_frequencies(chanspec[0],chanspec[1]),
                self.theta_space,
                self.tof_space
                )
            self.chanspec_seen.add(chanspec)
            self.svd_window[chanspec] = np.zeros((self.rx_pos.shape[0]*constants.n_sub[bw],self.pkt_window),
                                                 dtype=np.complex128)
            self.svd_roll[chanspec] = 0

        num_meas = H.shape[1]
        c_roll = self.svd_roll[chanspec]
        for n in range(num_meas):
            self.svd_window[chanspec][:, c_roll] = H[:,:,n].reshape((-1),order='F')

            c_roll += 1
            if(c_roll >= self.pkt_window):
                c_roll = 0
        self.svd_roll[chanspec] = c_roll

        H_flat = H.reshape((H.shape[0]*H.shape[1],H.shape[2]), order='F')
        null_space = np.linalg.svd(H_flat)[0][:,1:].T

        n_null = null_space.shape[0]
        n_rx = H.shape[1]
        n_sub= H.shape[0]
        n_tau = self.Tau[chanspec].shape[1]

        # numpy's matrix multiplication w/ extra leading dimensions isn't multithreaded so
        # we have to do this, otherwise will take 12x as long to do the matrix multiplication
        null_Tau = null_space.reshape((n_null*n_rx,n_sub)) @ self.Tau[chanspec]

        prof = self.Theta[chanspec] @ \
               null_Tau.reshape((n_null,n_rx,n_tau)).transpose((1,0,2)).reshape((n_rx,n_null*n_tau))
        prof = prof.reshape((prof.shape[0],n_null,n_tau))
        prof = np.reciprocal(np.linalg.norm(prof,axis=1)+eps)

        return self.theta_space[argmaxlocal(prof, thresh=0.99, exclude_borders=False)[0,0]], prof

class rx_svd_aoa_sensor:
    def __init__(self, rx_pos, theta_space, tof_space, pkt_window=40):
        self.thetas = {}
        self.taus = {}
        self.rx_pos = rx_pos
        self.theta_space = theta_space
        self.tof_space = tof_space
        self.pkt_window = pkt_window
        self.chanspec_seen = set()
        self.Theta = {}
        self.Tau = {}
        self.svd_window = {}
        self.svd_roll = {}
        self.prof_dim=2

    def __call__(self, H, chanspec):
        bw = chanspec[1]
        if chanspec not in self.chanspec_seen:
            self.Theta[chanspec], self.Tau[chanspec] = fft_mat(
                self.rx_pos,
                constants.get_channel_frequencies(chanspec[0],chanspec[1]),
                self.theta_space,
                self.tof_space
                )

            self.chanspec_seen.add(chanspec)
            self.svd_window[chanspec] = np.zeros((constants.n_sub[bw],self.rx_pos.shape[0],self.pkt_window),dtype=np.complex128)
            self.svd_roll[chanspec] = 0

        num_meas = H.shape[2]
        c_roll = self.svd_roll[chanspec]

        for n in range(num_meas):
            self.svd_window[chanspec][:,:,c_roll] = H[:,:,n]

            c_roll += 1
            if c_roll >= self.pkt_window:
                c_roll = 0
        self.svd_roll[chanspec] = c_roll

        U_csi = np.linalg.svd(self.svd_window[chanspec])[0][:,:,0].T

        prof = np.abs(self.Theta[chanspec] @ U_csi @ self.Tau[chanspec])
        return self.theta_space[argmaxlocal(prof, thresh=0.99, exclude_borders=False)[0,0]], prof

class fft_aoa_sensor:
    def __init__(self, rx_pos, theta_space, tof_space, valid_tx_ant=None):
        self.thetas = {}
        self.taus = {}
        self.rx_pos = rx_pos
        self.theta_space = theta_space
        self.tof_space = tof_space
        self.chanspec_seen = set()
        self.Theta = {}
        self.Tau = {}
        self.valid_tx_ant = valid_tx_ant
        self.prof_dim=2

        self.profile_tx_id = 0

    def __call__(self, H, chanspec):
        if chanspec not in self.chanspec_seen:
            self.Theta[chanspec], self.Tau[chanspec] = fft_mat(
                self.rx_pos,
                constants.get_channel_frequencies(chanspec[0],chanspec[1]),
                self.theta_space,
                self.tof_space
                )
            self.chanspec_seen.add(chanspec)

        H = H.transpose(2,1,0)

        aoas = []
        prof_to_ret = None

        tx_it = range(H.shape[0]) if self.valid_tx_ant is None else self.valid_tx_ant
        for idx in tx_it:
            # skip packets across transmit antennas which have zero information
            # last subcarrier carries some data interestingly
            if np.all(H[idx, :, :-1] == 0):
                continue

            prof = np.abs(self.Theta[chanspec] @ H[idx] @ self.Tau[chanspec])
            aoas.append(self.theta_space[argmaxlocal(prof, thresh=0.99, exclude_borders=False)[0,0]])
            if idx == self.profile_tx_id:
                prof_to_ret = prof

        aoa = np.angle(np.mean(np.exp(1.0j*np.asarray(aoas))))
        return aoa, prof_to_ret

class music_aoa_sensor_1d:
    def __init__(self, rx_pos, theta_space, pkt_window=40):
        self.rx_pos = rx_pos
        self.theta_space = theta_space

        self.pkt_window = pkt_window
        self.chanspec_seen = set()
        self.Theta = {}
        self.svd_window = {}
        self.svd_roll = {}
        self.prof_dim=1

    def __call__(self, H, chanspec):

        bw = chanspec[1]
        if chanspec not in self.chanspec_seen:
            self.Theta[chanspec], _ = fft_mat(
                self.rx_pos,
                constants.get_channel_frequencies(chanspec[0],chanspec[1]),
                self.theta_space,
                np.asarray([0])
                )
            self.chanspec_seen.add(chanspec)
            self.svd_window[chanspec] = np.zeros((self.pkt_window,self.rx_pos.shape[0],self.rx_pos.shape[0]),dtype=np.complex128)
            self.svd_roll[chanspec] = 0

        num_meas = H.shape[1]

        c_roll = self.svd_roll[chanspec]
        for n in range(num_meas):
            self.svd_window[chanspec][c_roll, :, :] = H[:,:,n].T @ H[:,:,n].conj()

            c_roll += 1
            if c_roll >= self.pkt_window:
                c_roll = 0
        self.svd_roll[chanspec] = c_roll

        U_csi = np.linalg.eig(np.sum(self.svd_window[chanspec], axis=0))[1][:,2:]

        prof = np.reciprocal(np.linalg.norm((self.Theta[chanspec] @ U_csi), axis=1) + eps)**2
        return self.theta_space[np.argmax(prof)], prof

class aoa_sensor_1d:
    def __init__(self, rx_pos, theta_space, pkt_window=40):
        self.rx_pos = rx_pos
        self.theta_space = theta_space

        self.pkt_window = pkt_window
        self.chanspec_seen = set()
        self.Theta = {}
        self.svd_window = {}
        self.svd_roll = {}
        self.prof_dim=1

        self.return_profile = False

    def __call__(self, H, chanspec):

        bw = chanspec[1]
        if chanspec not in self.chanspec_seen:
            self.Theta[chanspec], _ = fft_mat(
                self.rx_pos,
                constants.get_channel_frequencies(chanspec[0],chanspec[1]),
                self.theta_space,
                np.asarray([0])
                )
            self.chanspec_seen.add(chanspec)
            self.svd_window[chanspec] = np.zeros((self.pkt_window,self.rx_pos.shape[0],self.rx_pos.shape[0]),dtype=np.complex128)
            self.svd_roll[chanspec] = 0

        num_meas = H.shape[2]

        c_roll = self.svd_roll[chanspec]
        for n in range(num_meas):
            self.svd_window[chanspec][c_roll, :, :] = H[:,:,n].T @ H[:,:,n].conj()

            c_roll += 1
            if(c_roll >= self.pkt_window):
                c_roll = 0
        self.svd_roll[chanspec] = c_roll

        U_csi = np.linalg.eig(np.sum(self.svd_window[chanspec], axis=0))[1][:,0]
        prof = np.abs(self.Theta[chanspec] @ U_csi)

        return self.theta_space[np.argmax(np.abs(self.Theta[chanspec] @ U_csi))], prof

def spotfi_mat(rx, freqs, theta, d, len_rect, ht_rect):
    """
    Generates the steering matrix for all the specified values of AoA's and ToF's.
    Inputs:
      rx = AP antenna positions, n_rx x 2, each antenna should be x,y 
      freqs: subcarrier frequencies
      theta = possible values of Angle of Arrivals
      d = possible values of distances
      len_rect = number of spanning subcarriers
      ht_rect = number of spanning antennas
    Refer to SpotFi paper for further details on variable names
    """
    mean_freq = np.mean(freqs)
    diff_freq = np.mean(np.diff(freqs))
    ant_sep = np.mean(np.linalg.norm(np.diff(rx, axis=0), axis=1))

    assert ht_rect > 1, "the steering vector must span across two or more antennas"
    
    Phi = np.ones((ht_rect, len(theta)))*(1 + 0j)
    for i in range(1, ht_rect):
        Phi[i, :] = np.exp(1j*2*np.pi*(mean_freq/C_SPEED) \
                            *ant_sep*np.sin(theta))**i

    omega_mat = np.repeat(np.exp(1j*2*np.pi*diff_freq*d/C_SPEED),
                          len_rect)
    powers = np.arange(1, len_rect+1).repeat(len(d)).reshape(-1,len(d)).T.flatten()
    Omega = np.power(omega_mat, powers).reshape(-1, len_rect).T

    return np.kron(Phi, Omega)

def circulant(arr):
    toRet = np.zeros((len(arr), len(arr)), dtype=arr.dtype)
    for ii in range(len(arr)):
      toRet[:, ii] = np.roll(arr, ii)
    return toRet

def col_norm(matrix):
    toRet = np.zeros(matrix.shape[1], dtype=matrix.dtype)
    for ii in range(matrix.shape[1]):
      toRet[ii] = np.linalg.norm(matrix[:, ii])
    return toRet

class spotfi_sensor:
    def __init__(self, rx_pos, theta_space, tof_space):
        self.rx_pos = rx_pos
        self.theta_space = theta_space
        self.tof_space = tof_space

        self.chanspec_seen = set()
        self.Theta = {}

        self.steering_mat = {}


    def __call__(self, H, chanspec):
        """
        Generates the spotfi profile and returns the likelihood for the given values of AoA's and ToF's
        Inputs:
          H = csi matrix measurement, numpy.ndarray[n_sub x n_rx_ant x n_tx_ant]
          params = structure with some pre-defined parameters
        Notes: As compared to the matlab code:
          1. This implementation has an extra column in the smoothened matrix
          as the sliding window spans all the way to the end of the matrix
          2. It makes no prior assumption to number of antennas
        """
        csi_mat_shape = np.array(H.shape)
        n_sub = csi_mat_shape[0]
        n_rx_ant = csi_mat_shape[1]
        n_tx_ant = csi_mat_shape[2]
        len_rect = int(n_sub/2)
        ht_rect = int(n_rx_ant - 1)

        if chanspec not in self.chanspec_seen:
            freqs = constants.get_channel_frequencies(chanspec[0],chanspec[1]),
            self.steering_mat[chanspec] = spotfi_mat(self.rx_pos, freqs,
                                                     self.theta_space, self.tof_space,
                                                     len_rect, ht_rect)

        circ_crop = np.zeros((n_tx_ant, n_rx_ant, len_rect, n_sub - len_rect), dtype=np.complex128)
        for j in np.arange(n_tx_ant):
            for i in np.arange(n_rx_ant):
                circ_crop[j, i, :, :] = np.flipud(circulant(np.flip(H[:, i, j]))) \
                    [:len_rect, :(n_sub - len_rect)]

        csi_smooth = np.zeros((ht_rect*len_rect, 2*n_tx_ant*(n_sub - len_rect)))*(0 + 0j)
        for jj in range(0, n_tx_ant):
            csi_smooth[:, 2*jj*len_rect:(2*jj+1)*len_rect] = \
                circ_crop[jj//2, :-1].reshape(-1, circ_crop[jj//2, :-1].shape[2])
            csi_smooth[:, (2*jj+1)*len_rect:(2*jj+2)*len_rect] = \
                circ_crop[jj//2, 1:].reshape(-1, circ_crop[jj//2, 1:].shape[2])


        R = csi_smooth @ csi_smooth.conj().T
        [vals, vecs] = np.linalg.eig(R)
        vecs = vecs/col_norm(vecs)

        assert np.all(vals >= 0), "PSD matrix cannot have negative eigenvalues"
        idx = np.abs(vals) < SPOTFI_THRESH*np.max(np.abs(vals))
        profile = np.reciprocal(col_norm(vecs[:, idx].conj().T @ self.steering_mat[chanspec])**2 \
                                + EPS)
        profile = np.abs(profile.reshape((len(self.theta_space), len(self.tof_space))))
        return self.theta_space[argmaxlocal(profile, thresh=0.99, exclude_borders=False)[0,0]], profile


def aoa_steering_vec(rx,freqs,theta):
    #rx: AP antenna positions, n_rx x 2, each antenna should be x,y or theta,r.
    #freqs: subcarrier frequencies
    #theta: theta search space
    #d: d search space
    
    #returns: Theta, Tau steering matrices, pre conjugated.
    #so you can compute a profile as such:
    # prof = Theta @ H.T @ Tau
    
    #wave number
    k=2*np.pi*np.mean(freqs)/(3e8)
    rxrad = np.zeros_like(rx)
    #convert to radial coords

    rxrad[:,0]=np.arctan2(rx[:,1], rx[:,0])
    rxrad[:,1]=np.linalg.norm(rx, axis=1)
    #steering matrix for AoA
    theta = np.repeat((theta[:,np.newaxis]),4,1)
    #difference in theta between the incoming signal and receiver offset from origin
    th_d = theta - rxrad[np.newaxis,:,0]
    #compute relative phase offset from origin and turn to exponential
    Theta = np.exp(1.0j*k*rxrad[:,1]*np.cos(th_d))
    Theta = Theta[:,np.newaxis,:]
    Theta = np.repeat(Theta, freqs.shape[0], axis=1)
    return Theta
