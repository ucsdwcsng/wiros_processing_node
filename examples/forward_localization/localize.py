import numpy as np
import matplotlib.pyplot as plt
import tqdm
from os.path import join, abspath

'''

This example file uses pre-known AP locations and the AoAs measured at them to estimate the bot's position at each time step. This can be expanded to full co-optimization of the bot's trajectory (see https://wcsng.ucsd.edu/p2slam/) for a bot-based sensor fusion approach.

'''

'''

To generate your own data to use with this file, you would want to:

- Record a ROSBAG file using the CSI node at each AP while the bot is moving around and transmitting packets.

- Use scripts/generate_channel_files.py or the Processing Node to generate AoAs

- Time synchronize the AoAs across bots for data association (you can just choose the most recent packet at each bot timestamp)

- Mark down the AP locations in the bot's frame of reference as I've done in gnd/ .

If you don't know the AP's locations beforehand or want a more general/scalable approach, I'd suggest checking out P2SLAM, linked above. I'll upload example code for running p2slam shortly as well.

'''

# Helper functions to do the position estimation. I'm using a Levenberg-Marquardt estimator here since it performs a little better than a linear estimator in cases of bad measurements.
# But linear triangulation would probably work okay as well.
def fl_jacobian(p,aps,weights):
    #p: pos: xy coord
    #aps:size n_ap x 2
    D = np.zeros((aps.shape[0], 2))
    for ap_i in range(aps.shape[0]):
        pr = p - aps[ap_i]
        n = np.linalg.norm(pr)**2
        D[ap_i, 0] = -pr[1]/n + pr[0]/(1+n)
        D[ap_i, 1] = +pr[0]/n + pr[1]/(1+n)
    # if(np.any(np.isnan(D))):
    # print(D)
    return weights[:,np.newaxis]*D
def fl_loss(p,aps,ap_thetas,preds,weights):
    # if np.all(p == np.asarray([0,0])):
        # print("L")
    f = np.zeros((aps.shape[0]))
    for ap_i in range(aps.shape[0]):
        pr = p - aps[ap_i]
        th = ap_thetas[ap_i] + preds[ap_i]
        f[ap_i] = (np.arctan2(pr[1], pr[0]) - th+np.pi)%(2*np.pi)-np.pi
    return weights*f
def step(p,aps,ap_thetas,preds,weights,lam=1.0):
    D = fl_jacobian(p,aps,weights)
    D[D>1e3] = 1e3
    D[D<-1e3] = -1e3
    f = fl_loss(p,aps,ap_thetas,preds,weights)
    fl = np.linalg.norm(f)

    proj = D.T @ D + lam*np.identity(2)
    pnew = p - (np.linalg.inv(proj) @ D.T @ f)

    return pnew

def forward_localize(aps, ap_thetas, preds, weights = None, p_start=None, num_steps=100, lambda_gain=0.8, use_pbar=False):
    #aps: num_aps x 2, ap xy positions
    #ap_thetas: num_aps, ap angles
    #preds: num_aps, predicted bearings
    #p_start: xy position to start optimization
    #num_steps: maximum number of optimizaton steps
    #lambda_gain: gain of lambda on loss decrease(optimization aggressiveness)
    if p_start is None:
        p_start = np.asarray([0,0])
    if weights is None:
        weights = np.ones((aps.shape[0]))
    lam=10.0
    l_i=np.linalg.norm(fl_loss(p_start,aps,ap_thetas,preds,weights))
    k = 0
    p = p_start
    iter_ct = 0
    p_all = []
    while(iter_ct < num_steps):
        if lam > 10000:
            return p
        pnew= step(p,aps,ap_thetas,preds,weights,lam=lam)
        lnew = np.linalg.norm(fl_loss(pnew,aps,ap_thetas,preds,weights))
        if lnew < l_i:
            if lnew < 1e-2:
                return p
            lam *= lambda_gain
            iter_ct += 1
            p_all.append(p)
            p = pnew
            l_i = lnew
        else:
            lam *= 10

    return p


# Code control params - load 4 aps
aps = ['149', '151', '225', '227']

PATH = "/home/wcsng/data/forward_localization"

# Ground truth bot positions
bot_xy=np.load('/home/wcsng/data/forward_localization_4th/bot.npz')['xy']

# AP XY positions
ap_pos = np.zeros((len(aps),2))
# AP Theta
ap_th = np.zeros((len(aps)))
pred = []
rssi = []

for idx, a in enumerate(aps):
    # Contains AP positions and ground truth AoAs for debugging.
    ap_file = np.load(join(PATH, f'gnd/gnd_{a}.npz'))
    ap_pos[idx,:] = ap_file['xy']
    ap_th[idx] = ap_file['yaw']
    prediction_file = np.load(join(PATH,f'pred/{a}-synced_aoa.npz'))

    rssi.append(prediction_file['rssi'])

    # Real Predictions
    pred.append(prediction_file['aoa'])
    # Ground Truth
    #pred.append(ap_file['gnd'])

    ap_file.close()
    prediction_file.close()

pred =np.asarray(pred)
rssi = np.asarray(rssi)

loc = np.zeros((2,pred.shape[1]))
loc_lin = np.zeros_like(loc)

loc[:,0] = forward_localize(ap_pos,ap_th,pred[:,0])

# Weight measurements based on RSSI
weightsI = np.zeros((len(aps)))
weightsI[np.argmin(rssi[:,0])] = 0.0

weightArr = np.asarray([0.0,0.5,1.0,1.0])
for i in tqdm.trange(1,pred.shape[1]):
    weights = np.ones((len(aps)))
    ar = np.argsort(rssi[:,i])
    # This can be tinkered with, right now since we have 4-ap visibility in this
    # scenario I'm disregarding the weakest AP and treating second-weakest with 0.5 weight.
    weights[ar[0]] = 0.0
    weights[ar[1]] = 0.5

    # forward_localize.
    loc[:,i] = forward_localize(ap_pos, ap_th, pred[:,i], weights, p_start=loc[:,i-1])


err_aoa = np.linalg.norm(loc - bot_xy.T, axis=0)

c_aoa = np.sort(err_aoa)

plt.figure()
plt.plot(c_aoa, np.linspace(0,1,c_aoa.shape[0]), label='CDF Error of Location Estimates')
plt.grid()
plt.legend()
plt.show()

np.savez_compressed('pred_gnd_aoa.npz', aoa_based=loc, linear=loc_lin)
