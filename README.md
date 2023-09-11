# CSI tools

This node provides utilities for calculating Angle-of-Arrival (AoA) from input CSI messages. 

< Go back to the [index page](https://github.com/ucsdwcsng/WiROS)

Two scripts are provided which use these tools:

- `aoa_node.py` is a ROS node that takes in input CSI and publishes resulting AoA to a topic, as well as optionally publishing images of AoA profiles and plots of the channel.

- `generate_channel_files.py` is a script which takes in a rosbag file containing CSI messages and outputs a folder of numpy archives containing channels, timestamps, and profiles in a format designed for post-processing use with MATLAB or python. If you just want to use this script it is documented [below](#post-processing-a-bag-file).

These scripts support arbitrary (planar) antenna array configurations allowing for linear, square, etc. array shapes. Several AoA-estimation algorithms including spotfi, 2d-fft, and some averaging-based expansions of 2d-fft are present and the scripts are written in such a way that it is very easy to add new ones.

## Computing AoA

All of the methods we use to compute AoA have the same fundamental idea, which is to compare the space of steering vectors with the received channel, sort of like beamforming in reverse. We provide several algorithms out-of-the-box for different compute/accuracy/time-averaging considerations. For a primer on AoA as well as in-depth explanation of the algorithms we provide, please check out the documentation [here] TODO:link to final doc pdf, or add current version to github

## Installing the ROS package

Install the [`rf_msgs`](https://github.com/ucsdwcsng/rf_msgs) package if you haven't already. It provides CSI and AoA message formats used by this package.

Place this repo in a catkin workspace in the `src` directory along with `rf_msgs`. Then build with:
```
catkin build
```
or
```
catkin_make
```

## Running the Node

The node receives CSI messages on /csi and publishes aoa messages on /aoa. It also optionally publishes profile images on /prof.
The node is written in python3, so running via roslaunch may require a workaround on ROS Kinetic or Melodic. However a launch file is provided for Noetic, and older versions can still run the node from path via python3.

```
rosrun bearing_sensor aoa_node _name:=aoa_node --prof --color --algo {algorithm name}
```

## Parameters

### Algorithm-Related

- `algo` : The algorithm to use. Provided are:
  - `aoa_only`
  - `fft`
  - `rx_svd`
  - `full_svd`
  - `music`
  - `spotfi`

- `num_theta_steps` : The number of AoA values to search over.

- `theta_thresh` : The limits of the range of AoAs to search over in degrees, 0 pointing along the x axis.

- `num_d_steps` : The number of distance/ToF values to search over. Only relative ToF between energy peaks can be computed but this is still useful for differentiating multiple peaks.

- `d_thresh` : Limits of the range of ToF to search over in meters.

- `smoothing_window` : Number of past packets to average over for a given transmitter, only used in algorithms which do this smoothing. This should be determined by how fast you expect the channel to change and how fast the target devices are transmitting.

- `rssi_thresh` : Only evaluate CSI with RSSI above this limit. Filters out poor-quality measurements.

- `valid_tx_ant` : Which transmitters to use. Some devices may advertise 4 transmitters in the training field but will leave of the slots empty.

- `correct_tof_offset` : Try to shift TOF to 0 by fitting a complex exponential to subcarrier phase and then dividing it out. Helps remove CFO, NTS offset.

### Device-Related

- `comp` : Optional path to a [compensation folder](#collecting-compensation-data) or file. These are channel-specific files containing calibration values for the measured phase. If you give a path to a folder, then the node will automatically load compensation files from the folder specified corresponding to the channel of the received CSI. Compensation files should be stored in the format `{receiver IP}-{channel number}.npy`, e.g. `192.168.1.1-36.npy`. If you give a path to a .npy file, then the device will always load this compensation file. Note that in general you need different compensation files for different channels.

- `rx_position` : The position of the receiver antennas in the coordinate frame you wish to measure in. AoA is measured going counter-clockwise from the positive x-axis. Typically you would put the first antenna at the origin. More explanations as well as some example antenna array setups can be found in [antennas.md](antennas.md). Also note that SpotFi assumes standard uniform linear array, as the CSI-Smoothing algorithm isn't well-defined for other array shapes.

### Output-Related

- `publish_profile` : Publish an image of the profile on `/prof`. Can be viewed through RViz or similar.
- `publish_channel` : Publish channel magnitude and phase on `/channel`.
- `relative_phase` : If `True`, the published channel image will have the 1st antenna channel divided out.
- `color_profile` : Nicer looking profile image.
- `profile_tx_id` : Which transmitter field's profile should be displayed. Only meaningful for AoA algorithms that don't average transmitters together, like `fft`.
- `channel_tx_id` : Same as above for the channel plotting feature.

## Collecting Compensation Data

In order to accurately measure AoA, we need the relative phase between different pairs of antennas. However, due to differing wire lengths as well as filter effects from the onboard RF circuitry, each RX chain will have a phase bias which varies per subcarrier and per channel. So in order to accurately measure AoA, these biases need to be measured and removed from incoming data. The necessary phases to remove these biases can then be passed in the `comp` parameter and will be applied to incoming CSI by the ROS node.

### Static Compensation

This method provides very clean results but requires a [4-to-1 power splitter](https://www.minicircuits.com/pdfs/ZN4PD-642W+.pdf), 3 SMA 50-ohm terminators, about 55dB of attenuators, and a second AC86u. If you don't have these then you can follow the [dynamic compensation method](#dynamic-compensation-file).

The simplest way to measure the phase bias at the receiver is to ensure that the phase at each receiver is identical and measure the resulting phase. The idea is to ensure this is the case by injecting exactly the same signal to each receiver.

1. Connect the 4 receiver ports of the AC86u to the output of the power splitter. Ensure that you have the same length of cable between the board and each splitter port as you normally would between the board and each antenna.

2. Connect the attenuators to the input of the splitter, and one of the outputs of the other AC86u to the attenuators. Ideally you should terminate the other 3 antenna ports to cancel any crosstalk.

3. Start [CSI collection](https://github.com/ucsdwcsng/wiros_csi_node) at both the transmitter and receiver ends. The transmitter should have packet injection turned on and the receiver should set its MAC filter to the address the transmitter is injecting with.

4. Save the CSI data measured to a bag file, via `rosbag record /csi -O compensation_csi.bag`

5. Use the rosbag processing script to create the compensation file:
    ```
    rosrun csi_tools generate_static_compensation.py BAG_PATH
    ```

Compensation files follow the naming convention `{IP}-{chanspec}.npy`. NOTE: The above file conjugates the received CSI matrix. 

### Dynamic Compensation

This method allows you to compute compensation on-the-fly with only the ground-truth AoA of the incoming packets known. The below method for getting ground truth AoA is as described in the publication.

1.  Set up a device (need not be another AC86u) in the environment in a known location sending beacon packets.

2.  Attach the listening device to your robot in the configuration you normally use to collect AoAs.

3.  Slowly drive the robot in a small circle to collect CSI from a large angle diversity.

4.  Compute the ground-truth AoA using the estimated pose of the robot and the known transmitter location. Exact steps for doing this depend on the SLAM system / odometry you are using.

5.  Export the csi bag file to a .npz file using [`generate_channel_files.py`](scripts/generate_channel_files.py)

6.  Synchronize the computed ground AoAs to the received channels.

7.  Input the ground truth AoA from step 6 and the channels from step 5 into [`dynamic_compensation.py`](scripts/dynamic_compensation.py)

## Post-Processing a Bag File

We provide an example script, [`generate_channel_files.py`](scripts/generate_channel_files.py) to parse a rosbag file into several .mat or .npz files easy to use with MATLAB or python. It will create one file per transmitter per channel. It can be copied and edited to use the appropriate parameters for your data.

The output `.npz` files will have the following field:

- `H` : Raw channel data
- `rssi` : RSSI data
- `t` : ROS-Time of data
- `aoa` : Computed AoA
- `prof` : Computed image


## Algorithms

Several algorithms are provided, more will be added in the future. A brief description below.

***aoa_only***
Stacks all transmitters and subcarriers over the last 8 packets and takes SVD. This emulates a channel with 4 receivers and very many transmitters providing a good U matrix at low compute. Provides AoA only and does not publish a profile

***fft***
Standard 2D-FFT. Takes median AoA across the transmitters and optionally returns the profile of the last transmitter.

***rx_svd***
Takes the first principle component over the last 8 packets of each 4x32 submatrix across subcarriers, giving a 234x4x1 channel and computes fft. This provides a best trade-off between accuracy and compute efficiency. For more details, see [2]

***full_svd***
Stacks receivers across subcarriers and takes first principle component over the last 8 packets of the 936x32 matrix, giving a 936x1 = 234x4x1 channel and computes fft.

***music***
Stacks receivers across subcarriers and computes the 936 x 935 null space, then computes the reciprocal of the projection of the steering vectors onto the channel null space. Very slow.

***spotfi [1]***
Performs super-resolution (increasing the rank of the measurements matrix) across antennas and subcarriers to better resolve multipath components of a signal from the direct path. Additionally, performs MUSIC on the super-resolved measurements. NOTE: This is a compute intensive method unsuitable for real-time operations and does not work for non-linear antenna arrays. 

## Citations: 
1. Kotaru, Manikanta, et al. "Spotfi: Decimeter level localization using wifi." Proceedings of the 2015 ACM Conference on Special Interest Group on Data Communication. 2015.
2. "WAIS: Leveraging WiFi for Online and Resource-Efficient SLAM"
3. Blanco, Alejandro, et al. "Accurate ubiquitous localization with off-the-shelf ieee 802.11 ac devices." The 19th Annual International Conference on Mobile Systems, Applications, and Services (MobiSys 2021). 2021.
