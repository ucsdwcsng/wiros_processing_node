import numpy as np
import csi_utils.transform_utils as transform_utils

def music_compensation(H,g_aoa,init,rx,freqs,return_loss=True,verbose=False, M=None):
    '''
    channel vector convention is to stack subcarriers, then receivers. So the channel vector
    is size 936, with four blocks of 234 stacked together.
    H: Channel vectors, 936 x N
    g_aoa: Ground truth AoA, N
    init: Initial compensation vector, 936
    rx: Receiver positions, 4x2
    freqs: Subcarrier Frequencies, 234
    
    M: If you need to run this algorithm many times on the same data (to test initializations)
    You can pre-compute the inner product matrix as it is done below and pass it as an arg.
    '''
    
    
    # if the noise space wasn't precomputed then compute it here
    if M is None:
        # Uh = np.zeros((NOISE_DIM, H.shape[0],H.shape[1]), dtype=np.complex128)
        S = transform_utils.aoa_steering_vec(rx, freqs, g_aoa).transpose(0,2,1).reshape((g_aoa.shape[0],-1)).conj()
        '''
        #create U^H @ S (noise space adjusted for expected steering vector)
        for pkt in range(H.shape[0]):
            u,s,_ = np.linalg.svd(H[pkt])
            # if pkt == 100:
            #     pl.draw_channel(u[:,0].reshape((234,4,1), order='F'), title="First Eigenvector")
            Uh[:,pkt,:] = (u[:,1:4].conj().T*(s[1:,np.newaxis])) @ np.diag(S[pkt,:])
            # Uh[:,pkt,:] = (u[:,1:4].conj().T) @ np.diag(S[pkt,:])
        Uh /= np.max(np.abs(Uh))
        #create PSD matrix (Uh^H @ Uh)
        Uh = Uh.reshape((-1,Uh.shape[2]))
        '''
        
        for pkt in range(H.shape[0]):
            H[pkt,:,:] = np.diag(S[pkt,:]) @ H[pkt,:,:]
        H = H.transpose((1,2,0))
        H = H.reshape((H.shape[0],-1))
        u,s,_ = np.linalg.svd(H)
        
        s /= np.max(s)
        u = u[:,1:]#/s[np.newaxis,1:]
        
        Uh = u.conj().T
        
    
        M = Uh.conj().T @ Uh

    #c is the compensation angles, phi is the comp. values (complex)
    c = init
    phi = np.exp(1.0j*c)

    N_runs = 600
    l = np.zeros(N_runs)
    lam = np.identity((M.shape[0]))
    lam_exit = False
    diff_exit = False
    # pbar = tqdm.trange(N_runs)
    for N in range(N_runs):
        #re-compute phi
        phi = np.exp(1.0j*c)
        
        #evaluate phi^H @ M @ phi (func we are trying to minimize)
        f = np.real(phi.conj().T @ M @ phi)#[0,0]
        #current loss
        l[N] = f

        #wiringer derivatives - http://dsp.ucsd.edu/~kreutz/PEI-05%20Support%20Files/complex_derivatives.pdf
        
        #derivative of phi^h @ M @ phi is is M* phi* 
        #conjugate derivative is M phi
        
        #full derivative is df/dc + df/dc* - https://math.stackexchange.com/questions/2635763/the-derivative-of-complex-quadratic-form
        
        # = dphi*/dc* @ df/dphi* + dphi/dc @ df/dphi
        D = np.real(np.diag(-1.0j*phi.conj()[:,0]) @ M @ phi + np.diag(1.0j*phi[:,0]) @ M.T @ phi.conj()).T
        
        #since func is real valued it simplifies to this
        # D = 2*np.real(np.diag(1.0j*phi[:,0]) @ M.T @ phi.conj().T)

        #update c
        c_n = c - np.linalg.solve(D.T @ D + lam, D.T @ f)
        phi_n = np.exp(1.0j*c_n)
        #new loss
        f_n = np.real(phi_n.conj().T @ M @ phi_n)#[0,0]
        
        #levenberg-marquardt
        while f_n >= f:
            if lam[0,0] > 1e20:
                lam_exit = True
                break
            lam *= 10
            c_n = c - np.linalg.solve(D.T @ D + lam, D.T @ f)
            phi_n = np.exp(1.0j*c_n)
            f_n = np.real(phi_n.conj().T @ M @ phi_n)#[0,0]
            print(f"{N}/{N_runs}: Loss:{f_n[0,0]: 0.5f}, \u03BB:{lam[0,0]}")
            # pbar.set_description(f"{f_n[0,0]: 0.1f}, {lam[0,0]}")
        if f - f_n < 1e-3:
            diff_exit = True
        if lam_exit or diff_exit:
            break
        
        lam *= 0.7
        c = np.angle(phi_n)
        print(f"{N}/{N_runs}: Loss:{f_n[0,0]: 0.5f}, \u03BB:{lam[0,0]}")
    if return_loss:
        return [np.exp(1.0j*c), f_n[0,0]]
    else:
        return np.exp(1.0j*c)
