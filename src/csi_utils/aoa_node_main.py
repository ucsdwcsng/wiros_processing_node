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
import csi_utils.io_utils as io_utils
from rf_msgs.msg import Wifi, Bearing
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
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
        self.pub_rel_channel = False
        self.accept_color = cm.get_cmap('magma')

        self.aoa_sensors = {}
        self.macs = set() # set of MAC addresses

        self.comp_path = None
        self.use_comp_folder = True
        self.comp = None # compensation to be applied to RAW CSI values

        self.apply_nts = True

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
        self._bearing_pose_pub = None

        #values filled in on message process
        self.last_channel = None
        self.last_rssi = None
        self.last_mac = None
        self.last_aoa = None

    def algo_selector(self, sensor):
        if self.algo == 'full_svd':
            sensor = transform_utils.full_svd_aoa_sensor(self.rx_position,
                                                         self._theta_range, self._tau_range,
                                                         self.pkt_smoothing_window, valid_tx_ant=self.valid_tx_ant)
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


    def draw_prof_image(self, prof):
        if self.use_color:
            profim = (self.accept_color(prof/np.max(prof))[:,:,:3] * 255).astype(np.uint8)
            peak_idx = int((self._theta_range.shape[0]-1)*(self.last_aoa - self._theta_range[0])/(self._theta_range[-1] - self._theta_range[0]))
            profim[peak_idx,:,:] = [255,0,0]
        else:
            profim = (255*prof/np.max(prof)).astype(np.uint8)

        fig = Figure(figsize=(10, 10))
        canvas = FigureCanvas(fig)
        axs = fig.add_subplot(111)
        
        axs.imshow(np.flipud(profim))
        axs.grid(color="white", linewidth=2)
        axs.set_xlabel("Relative Distance (m)", fontsize=20)
        axs.set_ylabel("Angle of Arrival (deg)", fontsize=20)

        axs.set_xticks(np.linspace(0, profim.shape[0], self.num_d_steps//20))
        axs.set_xticklabels(np.linspace(self.d_thresh[0], self.d_thresh[1], self.num_d_steps//20))
        axs.set_yticks(np.linspace(0, profim.shape[0], self.num_d_steps//20))
        axs.set_yticklabels(np.linspace(self.theta_thresh[1], self.theta_thresh[0], self.num_d_steps//20))

        axs.tick_params(axis='both', which='major', labelsize=18)

        fig.set_tight_layout(True)


        canvas.draw()  # draw the canvas, cache the renderer
        
        width, height = fig.get_size_inches() * fig.get_dpi()
        image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape((int(height), int(width), 3))
        return image

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
        tic = time.time()
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
        if self.comp_path is not None:
            if self.use_comp_folder:
                comp_spec=(msg.rx_id,msg.chan)
                if comp_spec not in self.comp.keys():
                    spec_file=join(self.comp_path, f"{comp_spec[0]}-{comp_spec[1]}.npy")
                    if os.path.isfile(spec_file):
                        self.comp[comp_spec] = np.load(spec_file)
                    else:
                        rospy.logerr(f"Tried to load compensation for {comp_spec[0]} on channel {comp_spec[1]}\nbut {spec_file} does not exist. Skipping for now.")
                        self.comp[comp_spec] = 1.0
                self.last_channel = pipeline_utils.extract_csi(msg, self.comp[comp_spec], self.apply_nts, self.valid_tx_ant)
            else:
                self.last_channel = pipeline_utils.extract_csi(msg, self.comp, self.apply_nts, self.valid_tx_ant)
        else:
            self.last_channel = pipeline_utils.extract_csi(msg, None, self.apply_nts, self.valid_tx_ant)
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

        toc = time.time()
        print(f"extract time: {toc - tic:.3e}")

        tic = time.time()
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

        toc = time.time()
        print(f"publish time: {toc - tic:.3e}")

        tic = time.time()
        # Publish the AoA-ToF profile
        if prof is not None and self.pub_prof:
            if self.aoa_sensors[mac].prof_dim == 2:

                if len(prof.shape) == 3:
                    prof = prof[:,:,self.prof_tx_id]
<<<<<<< HEAD
                if self.use_color:
                    profim = (self.accept_color(prof/np.max(prof))[:,:,:3] * 255).astype(np.uint8)
                    peak_idx = int((self._theta_range.shape[0]-1)*(self.last_aoa - self._theta_range[0])/(self._theta_range[-1] - self._theta_range[0]))
                    profim[peak_idx,:,:] = [255,0,0]
                    profim = np.flipud(profim)
                    im_msg = io_utils.image_message(profim, msg.header.stamp, 'rgb8')
                    self._prof_pub.publish(im_msg)
                else:
                    profim = (255*prof/np.max(prof)).astype(np.uint8)
                    im_msg = io_utils.image_message(profim, msg.header.stamp, 'mono8')
                    self._prof_pub.publish(im_msg)
            else:
=======

                prof_im = self.draw_prof_image(prof)
                im_msg = io_utils.image_message(prof_im, msg.header.stamp, 'rgb8')
                self._prof_pub.publish(im_msg)
            else: 
>>>>>>> f4fc3c10ad1554aa6c050cf575ef1b66d8161be4
                profim = io_utils.draw_1d_profile(prof, self._theta_range)
                im_msg = io_utils.image_message(profim, msg.header.stamp, 'rgb8')
                self._prof_pub.publish(im_msg)
                
            # Publish a pose message to visualize the bearing 
            bearing_pose = Pose()
            bearing_pose.orientation.x = 0   
            bearing_pose.orientation.y = 0    
            bearing_pose.orientation.z = np.sin(self.last_aoa/2)   
            bearing_pose.orientation.w = np.cos(self.last_aoa/2)

            bearing_pose_msg = PoseStamped()
            bearing_pose_msg.pose = bearing_pose
            bearing_pose_msg.header = msg.header
            bearing_pose_msg.header.frame_id = "map"
            self._bearing_pose_pub.publish(bearing_pose_msg)   
        
        toc = time.time()
        print(f"prof time: {toc - tic:.3e}")

        tic = time.time()
        # Publish the magnitude and phase of channel
        if self.pub_channel and self.last_channel is not None:
            if self.pub_rel_channel:
                channel_im = io_utils.draw_channel_image(self.last_channel / self.last_channel[:,0,np.newaxis,:])
            else:                                    
                channel_im = io_utils.draw_channel_image(self.last_channel)
            im_msg = io_utils.image_message(channel_im, msg.header.stamp, 'rgb8')
            self._channel_pub.publish(im_msg)

        # add AP marker into the world
        ap_marker_msg = Marker()
        ap_marker_msg.header.frame_id = "map"
        ap_marker_msg.header.stamp = msg.header.stamp
        ap_marker_msg.id = 0

        ap_marker_msg.pose.position.x = -0.1
        ap_marker_msg.pose.position.y = -0.75
        ap_marker_msg.pose.position.z = 0
        ap_marker_msg.pose.orientation.x = 0.0
        ap_marker_msg.pose.orientation.y = 0.0
        ap_marker_msg.pose.orientation.z = np.sin(-45*np.pi/180)
        ap_marker_msg.pose.orientation.w = np.cos(-45*np.pi/180)
        ap_marker_msg.scale.x = 0.05
        ap_marker_msg.scale.y = 0.05
        ap_marker_msg.scale.z = 0.05

        ap_marker_msg.color.a = 1
        ap_marker_msg.color.r = 0.6
        ap_marker_msg.color.g = 0.6
        ap_marker_msg.color.b = 0.6

        ap_marker_msg.type = Marker.MESH_RESOURCE;
        ap_marker_msg.mesh_resource = "package://csi_tools/meshes/ap_simple.stl"
        self._ap_marker_publisher.publish(ap_marker_msg)

        toc = time.time()
        print(f"chan time: {toc - tic:.3e}")
        rospy.loginfo("RSSI %f, AOA %f deg", msg.rssi, self.last_aoa*180/np.pi)

        
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

        self._prof_pub = rospy.Publisher('/prof', Image, queue_size=3)
        self._channel_pub = rospy.Publisher('/channel', Image, queue_size=3)
        self._bearing_pose_pub = rospy.Publisher('/bearing_pose', PoseStamped, queue_size=3)
        self._ap_marker_publisher = rospy.Publisher('/ap_marker', Marker, queue_size=1)

        if self.comp is None:
            rospy.logwarn("No compensation provided.")

        rospy.Subscriber('/csi', Wifi, self.csi_callback, queue_size=1)
        save_service = rospy.Service('savecsi', SaveChannel, self.save_channel)
        rospy.spin()
