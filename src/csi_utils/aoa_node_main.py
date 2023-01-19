import matplotlib.pyplot as plt
import rospy
import rosbag
import numpy as np
import sys
import time
from os.path import join
import os
import csi_utils.constants as constants
import csi_utils.transform_utils as transform_utils
import csi_utils.pipeline_utils as pipeline_utils
from rf_msgs.msg import Wifi, Bearing
from sensor_msgs.msg import Image
from csi_tools.srv import SaveChannel, SaveChannelResponse
import traceback

import matplotlib.cm as cm
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

class aoa_node:
    def __init__(self):
        #global params - modify these in code before calling run()
        self.use_color = False
        self.pub_prof = False
        self.pub_channel = False
        self.prof_tx_id = 0
        self.chan_tx_id = 0
        self.algo = None
        self.accept_color = cm.get_cmap('magma')

        self.aoa_sensors = {}
        self.macs = set() # set of MAC addresses
        self.comp = None # compensation to be applied to RAW CSI values
<<<<<<< HEAD
=======
        self.apply_nts = True
>>>>>>> 454f0030aade08df023d6bedbf5f260f67468c4c

        self.csi_export_mac_filter = None
        self.num_save_csi = 0
        self.save_csi_buf = []
        self.save_rssi_buf = []
        self.save_csi_path = ""
        
        # Passed in from launch file
        self.theta_thresh = None
        self.d_thresh = None
        self.rx_position = None
        self.valid_tx_ant = None
        self.pkt_smoothing_window = None
        self.rssi_threshold = None

        #should be dynamically created on run
        self._theta_range = None
        self._tau_range = None
        self._aoa_pub = None
        self._prof_pub = None
        self._channel_pub = None

        #values filled in on message process
        self.last_channel = None
        self.last_rssi = None
        self.last_mac = None
        self.last_aoa = None

    def algo_selector(self, sensor):
        if self.algo == 'full_svd':
            sensor = transform_utils.full_svd_aoa_sensor(self.rx_position, self.valid_tx_ant,
                                                         self._theta_range, self._tau_range,
                                                         self.pkt_smoothing_window)
        elif self.algo == 'rx_svd':
            sensor = transform_utils.rx_svd_aoa_sensor(self.rx_position, self._theta_range,
                                                       self._tau_range, self.pkt_smoothing_window)
        elif self.algo == 'aoa_only':
            sensor = transform_utils.aoa_sensor_1d(self.rx_position, self._theta_range,
                                                   self.pkt_smoothing_window)
        elif self.algo == 'fft':
            sensor = transform_utils.fft_aoa_sensor(self.rx_position,
                                                    self._theta_range, self._tau_range)
        elif self.algo == 'music':
            sensor = transform_utils.full_music_aoa_sensor(self.rx_position, self._theta_range,
                                                           self._tau_range, self.pkt_smoothing_window)
        elif self.algo == 'aoa_music':
            sensor = transform_utils.music_aoa_sensor_1d(self.rx_position, self._theta_range,
                                                         self.pkt_smoothing_window)
        elif self.algo == 'spotfi':
            sensor = transform_utils.spotfi_sensor(self.rx_position, self._theta_range,
                                                   self._tau_range)
        elif self.algo is None:
            rospy.logerr(f"Incorrect Algorithm {self.algo} chosen")

        return sensor

    #publish an image
    def publish_image(self, im_arr, t, id, im_type, publisher):
        
        ros_image = Image(encoding=im_type)
        
        # Create the header
        ros_image.header.stamp = t
        ros_image.header.frame_id = id
        
        # Fill the image data 
        ros_image.height = im_arr.shape[0]
        ros_image.width = im_arr.shape[1]
        ros_image.data = im_arr.ravel().tobytes() # or .tostring()
        if im_type == "rgb8":
            ros_image.step= 3*ros_image.width
        else:
            ros_image.step= ros_image.width
        
        publisher.publish(ros_image)

    def export_last_channel(self):
        if self.csi_export_mac_filter is not None and self.last_mac != self.csi_export_mac_filter:
            return
        self.save_csi_buf.append(self.last_channel)
        self.save_rssi_buf.append(self.last_rssi)
        self.num_save_csi -= 1
        rospy.loginfo(f"Saving {self.num_save_csi} more channels.")
        if(self.num_save_csi < 1):
            save_csi = np.asarray(self.save_csi_buf)
            save_rssi = np.asarray(self.save_rssi_buf)
            np.savez(self.save_csi_path, H=save_csi, rssi=save_rssi)
            self.num_save_csi = 0
            self.save_csi_buf = []
            self.save_rssi_buf = []
        return        

        
    #callback
    def csi_callback(self, msg):

        if msg.rssi < self.rssi_threshold:
            return
        
        mac = pipeline_utils.mac_to_str(tuple(msg.txmac))
        bw = msg.bw*1e6

        rospy.loginfo(f"CSI from %s", mac)

        if mac not in self.macs:
            self.macs.add(mac)
            self.aoa_sensors[mac] = self.algo_selector(mac)
            
            if self.pub_prof:
                self.aoa_sensors[mac].profile_tx_id = self.prof_tx_id

<<<<<<< HEAD
        self.last_channel = pipeline_utils.extract_csi(msg, self.comp, self.valid_tx_ant)
=======
        self.last_channel = pipeline_utils.extract_csi(msg, self.comp, self.apply_nts, self.valid_tx_ant)
>>>>>>> 454f0030aade08df023d6bedbf5f260f67468c4c
        self.last_mac = mac
        self.last_rssi = msg.rssi
        
        
        if self.num_save_csi > 0:
            self.export_last_channel()

        
        try:
            self.last_aoa, prof = self.aoa_sensors[mac](self.last_channel, (msg.chan, bw))
        except Exception as e:
            prof = None
            rospy.logerr("Error in computing AoA and profile")
            rospy.logerr(e)
            traceback.print_exc()
            return

        # TODO Add AoD

        angle_msg = Bearing()
        angle_msg.header = msg.header
        angle_msg.header.stamp = rospy.get_rostime()
        angle_msg.n_tx = msg.n_cols
        angle_msg.n_rx = msg.n_rows
        angle_msg.seq = msg.seq_num
        angle_msg.rssi = [int(msg.rssi)]
        angle_msg.txmac = msg.txmac
        angle_msg.ap_id = msg.ap_id
        angle_msg.aoa = [self.last_aoa]
        
        self._aoa_pub.publish(angle_msg)

        # Publish the AoA-ToF profile
        if prof is not None and self.pub_prof:
            if self.use_color:
                profim = (self.accept_color(prof/np.max(prof))[:,:,:3] * 255).astype(np.uint8)
                peak_idx = int((self._theta_range.shape[0]-1)*(self.last_aoa - self._theta_range[0])/(self._theta_range[-1] - self._theta_range[0]))
                profim[peak_idx,:,:] = [255,0,0]
                self.publish_image(profim, msg.header.stamp, msg.header.frame_id, 'rgb8',
                                   self._prof_pub)
            else:
                profim = (255*prof/np.max(prof)).astype(np.uint8)
                self.publish_image(profim, msg.header.stamp, msg.header.frame_id, 'mono8',
                                   self._prof_pub)

        # Publish the magnitude and phase of channel
        if self.pub_channel and self.last_channel is not None:
            fig = Figure(figsize=(8, 10))
            canvas = FigureCanvas(fig)
            axs = fig.subplots(2, 1)

            # plot magnitude
            axs[0].set_xlabel("Subcarrier indices")
            axs[0].set_ylabel("Magnitude (dB)")
            for ii in range(self.last_channel.shape[1]):
                axs[0].plot(20*np.log10(np.abs(self.last_channel[:, ii, self.chan_tx_id])),
                            label=f"Rx Ant {ii}")
            axs[0].legend()

            # plot phase
            axs[1].set_xlabel("Subcarrier indices")
            axs[1].set_ylabel("Phase (deg)")
            for ii in range(self.last_channel.shape[1]):
                axs[1].plot(np.unwrap(np.angle(self.last_channel[:, ii, self.chan_tx_id]))*180/np.pi,
                            label=f"Rx Ant {ii}")
            axs[1].legend()

<<<<<<< HEAD
            fig.set_tight_layout(True)
=======
            plt.tight_layout()
>>>>>>> 454f0030aade08df023d6bedbf5f260f67468c4c
            canvas.draw()  # draw the canvas, cache the renderer

            width, height = fig.get_size_inches() * fig.get_dpi()
            image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape((int(height), int(width), 3))

            self.publish_image(image, msg.header.stamp, msg.header.frame_id, 'rgb8',
                               self._channel_pub)

        rospy.loginfo("RSSI %f, AOA %f", msg.rssi, self.last_aoa)

        
    def save_channel(self, req):
        if req.num_channels is None or req.num_channels < 1:
            self.num_save_csi = 1
        else:
            self.num_save_csi = req.num_channels
        if req.mac == '':
            self.csi_export_mac_filter = None
        else:
            self.csi_export_mac_filter = req.mac
        self.save_csi_path = os.path.abspath(req.filename)
        result = SaveChannelResponse()
        result.result = f"Saving CSI to {self.save_csi_path}"
        nchannels = self.num_save_csi
        while(self.num_save_csi > 0):
            time.sleep(0.1)
            continue
        return result


        
    #start the node
    def start(self):
        self._theta_range = np.linspace(self.theta_thresh[0]*np.pi/180, self.theta_thresh[1]*np.pi/180,
                                        self.num_theta_steps)
        self._tau_range = np.linspace(self.d_thresh[0], self.d_thresh[1], self.num_d_steps)
        self._aoa_pub = rospy.Publisher('/bearing', Bearing, queue_size=3)

        if self.pub_prof and self.algo in constants.no_prof_algos:
            rospy.logerr(f"pub_prof set to True but algo {self.algo} does not generate a profile")
            rospy.loginfo("Turning off pub_prof")
            self.pub_prof = False

        self._prof_pub = rospy.Publisher('/prof', Image, queue_size=3)
        self._channel_pub = rospy.Publisher('/channel', Image, queue_size=3)

        if self.comp is None:
            rospy.logwarn("No compensation provided.")

        rospy.Subscriber('/csi', Wifi, self.csi_callback)
        save_service = rospy.Service('savecsi', SaveChannel, self.save_channel)
        rospy.spin()
