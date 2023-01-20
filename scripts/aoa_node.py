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

    compensate_channel = rospy.get_param("~compensate_channel", False)
    aoa.apply_nts = rospy.get_param("~correct_tof_offset", False)

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

    if compensate_channel:
        try:
            aoa.comp = np.load(aoa.comp_file)
        except:
            aoa.comp = None
            rospy.logwarn(f"Failed to read compensation file at {aoa.comp_file}")
    
    #start node
    aoa.start()
