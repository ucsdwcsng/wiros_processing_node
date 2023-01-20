Below are some example antenna setups.

Uniform Linear Array (Normal 0,1,2,3 spacing, distance between antennas=lambda/2 on channel 42)

```
<rosparam param="rx_position"> [0,  0,
    0,  0.026,
    0,  0.052,
    0,  0.078] </rosparam>
```
```
3
2
1
0  --X-Axis-->
```
--

The same array with a more natural antenna placement for the RT-AC86u. It is easier to put the antennas in this order due to the locations of the SMA ports on the physical device.
```
<rosparam param="rx_position"> [0,  0,
    0,  0.052,
    0,  0.026,
    0,  0.078] </rosparam>
```
```
3
1
2
0 --X-Axis-->
```

--

A Square antenna array that can sense the full 360-degree range at the cost of slightly reduced performance. We find that the optimal square side length to avoid aliasing problems is about 3*lambda/8, so to ensure the array works at the highest 5GHz frequencies we set the side length to 19mm.
```
<rosparam param="rx_position"> [0,  0,
    0,  0.019,
    -0.019,  0.0,
    -0.019,  0.019] </rosparam>
```
```
3  1

2  0  --X-Axis-->
```
