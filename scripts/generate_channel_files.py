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

<<<<<<< HEAD
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
                
=======
if __name__ == '__main__':
    print(os.getcwd())
    parse = argparse.ArgumentParser()
    parse.add_argument("bag", help="Input CSI bag file")
    parse.add_argument("-o", "--out", help="Directory to output processed data", default = os.getcwd())
    parse.add_argument("-a", "--algo", default=None, help="AoA extraction algorithm to use")
    parse.add_argument("-c","--comp", help="Compensation File to use")
    parse.add_argument("-p", "--prof", help="Publish AoA Profiles as a topic", action='store_true')
    parse.add_argument("--create-comp", help="Generate a compensation file from static compensation data",action='store_true')
    args,_ = parse.parse_known_args()

    if not os.path.isfile(args.bag):
        rospy.logfatal(f"{args.bag} does not exist.")
        exit(1)

    bag = rosbag.Bag(args.bag)
    out = args.out
    create_comp = args.create_comp
    if args.comp is not None:
        comp = np.load(args.comp)
    else:
        comp = None
        if not create_comp:
            print("warning: no comp file provided")
    
    if args.algo is not None:
        use_aoa = True

        aoa_algo = args.algo
    else:
        use_aoa = False


    
    if args.prof:
        if not use_aoa:
            print("profile saving set but no algorithm specified.")
            exit(1)
        save_prof = True
    else:
        save_prof = False


    channels = {}
    times = {}
    aoas = {}
    rssis = {}
    profs = {}
    aoa_sensors = {}
    rx = None
    for topic, msg, t in tqdm.tqdm(bag.read_messages('/csi')):
        csi = pipeline_utils.extract_csi(msg, comp)
        #assuming this does not change
        rx = msg.rx_id
        mac = pipeline_utils.mac_to_str(tuple(msg.txmac))

        # mac = '00:6b:f1:4c:ed:80'
        key = f"{mac}-{msg.chan}-{msg.bw}"

        if key not in channels.keys():
            channels[key] = []
            times[key] = []
            rssis[key] = []

            if use_aoa:

                #square array
                sqside = 0.00945*2
                rxpos = np.asarray([[0,0],[0,sqside],[-sqside,0],[-sqside,sqside]])

                if aoa_algo == 'full_svd':
                    aoa_sensors[key] = transform_utils.full_svd_aoa_sensor(rxpos, np.linspace(-np.pi,np.pi,360), np.linspace(-40,40,120), 40)
                elif aoa_algo == 'rx_svd':
                    aoa_sensors[key] = transform_utils.rx_svd_aoa_sensor(rxpos, np.linspace(-np.pi,np.pi,360), np.linspace(-40,40,120), 40)
                elif aoa_algo == 'aoa_only':
                    aoa_sensors[key] = transform_utils.aoa_sensor_1d(rxpos, np.linspace(-np.pi,np.pi,360), 40)
                elif aoa_algo == 'fft':
                    aoa_sensors[key] = transform_utils.fft_aoa_sensor(rxpos, np.linspace(-np.pi,np.pi,360), np.linspace(-40,40,120))
                elif aoa_algo == 'music':
                    aoa_sensors[key] = transform_utils.full_music_aoa_sensor(rxpos, np.linspace(-np.pi,np.pi,360), np.linspace(-40,40,120), 40)
                elif aoa_algo == 'aoa_music':
                    aoa_sensors[key] = transform_utils.music_aoa_sensor_1d(rxpos, np.linspace(-np.pi,np.pi,360), 40)
                else:
                    print(f"Invalid AoA {aoa_algo}")
                print(aoa_algo)
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
            profs[key].append(res[1])

    if create_comp:
        maxk = ''
        maxlen = 0
        for key in channels:
            if len(channels[key]) > maxlen:
                maxk = key
                maxlen = len(channels[key])
        if maxk == '':
            rospy.logfatal("No CSI in bag file")
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
    else:
        for key in channels:
            
            f_dict = {'H':np.asarray(channels[key]),
                      't':np.asarray(times[key]),
                      'rssi':np.asarray(rssis[key]),
            }
            
            if use_aoa:
                f_dict['aoa'] = np.asarray(aoas[key])
                if save_prof:
                    f_dict['prof'] = np.asarray(profs[key])
                    
                    np.savez(join(out, f"{key}.npz"), **f_dict)
                    print(f"Created {join(out, f'{key}.npz')}")
                    
    exit(0)
>>>>>>> 454f0030aade08df023d6bedbf5f260f67468c4c
