#! /usr/bin/env python3
import rospy
from csi_utils.aoa_node_main import aoa_node
import numpy as np

EPS = np.finfo('float').eps


if __name__ == '__main__':
    rospy.init_node('aoa_node', anonymous=True)

    #create node wrapper
    aoa = aoa_node()
    
    #setup node options
    aoa.algo = rospy.get_param("~algo", "fft")

    aoa.comp_file = rospy.get_param("~comp", None)
<<<<<<< HEAD
    is_conjugated = rospy.get_param("~comp_is_conjugated", False)
    is_phase_only = rospy.get_param("~comp_is_phase_only", False)
=======
    compensate_channel = rospy.get_param("~compensate_channel", False)
    aoa.apply_nts = rospy.get_param("~correct_tof_offset", False)
>>>>>>> 454f0030aade08df023d6bedbf5f260f67468c4c

    aoa.pub_prof = rospy.get_param("~publish_profile", True)
    aoa.pub_channel = rospy.get_param("~publish_channel", True)
    aoa.use_color = rospy.get_param("~color_profile", True)
    aoa.prof_tx_id = rospy.get_param("~profile_tx_id", 0)
    aoa.chan_tx_id = rospy.get_param("~channel_tx_id", 0)


    aoa.num_theta_steps = rospy.get_param("~num_theta_steps", 180)
    aoa.num_d_steps = rospy.get_param("~num_d_steps", 100)
    aoa.pkt_smoothing_window = rospy.get_param("~smoothing_window", 40)

    aoa.theta_thresh = rospy.get_param("~theta_thresh", [-180, 180])
    aoa.d_thresh = rospy.get_param("~d_thresh", [-10, 40])

    aoa.rssi_threshold = rospy.get_param("~rssi_thresh", -65)
    # Will use all TX if left as None
    aoa.valid_tx_ant = rospy.get_param("~valid_tx_ant", None)

<<<<<<< HEAD
    # array_type = rospy.get_param("~array_type", "square")
    # if array_type == "square":
    #     assert aoa.algo != 'spotfi', "Spotfi does not support non-linear antenna array"
    #     ant_sep = rospy.get_param("~antenna_sep", 0.185)
    #     aoa.rx_position = np.asarray([[0,0],[0,ant_sep],[-ant_sep,0],[-ant_sep,ant_sep]])
    # elif array_type == "linear":
    #     ant_sep = rospy.get_param("~antenna_sep")
    #     num_antennas = rospy.get_param("~num_antennas")
    #     aoa.rx_position = np.asarray([[0, -ii*ant_sep] for ii in range(num_antennas)])
    # elif array_type == "custom":
    #     assert aoa.algo != 'spotfi', "Spotfi does not support non-linear antenna array"
    #     aoa.rx_position = np.array(rospy.get_param("~rx_position")).reshape((-1, 2))
    # else:   
    #     rospy.logerr("Incorrect Array type provided, must be 'square', 'linear', or 'custom'")
    aoa.rx_position = np.asarray(rospy.get_param("~rx_position", [0,  0,
                                        0,  0.052,
                                        0,  0.026,
                                        0,  0.078])).reshape((-1,2))
    
    try:
        aoa.comp = np.load(aoa.comp_file)
    except:
        aoa.comp = None
        rospy.logwarn(f"Failed to read compensation file at {aoa.comp_file}")
=======
    if compensate_channel:
        try:
            aoa.comp = np.load(aoa.comp_file)
        except:
            aoa.comp = None
            rospy.logwarn(f"Failed to read compensation file at {aoa.comp_file}")
>>>>>>> 454f0030aade08df023d6bedbf5f260f67468c4c
    
    #start node
    aoa.start()
