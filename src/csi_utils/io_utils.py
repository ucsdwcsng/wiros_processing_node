import numpy as np
from sensor_msgs.msg import Image
import matplotlib.cm as cm
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

    #publish an image
def image_message(im_arr, t, im_type):
    
    ros_image = Image(encoding=im_type)
        
    # Create the header
    ros_image.header.stamp = t
    ros_image.header.frame_id = ""
    
    # Fill the image data 
    ros_image.height = im_arr.shape[0]
    ros_image.width = im_arr.shape[1]
    ros_image.data = im_arr.ravel().tobytes() # or .tostring()
    if im_type == "rgb8":
        ros_image.step= 3*ros_image.width
    else:
        ros_image.step= ros_image.width
            
    return ros_image

def draw_channel_image(channel):
    fig = Figure(figsize=(10, 10))
    canvas = FigureCanvas(fig)
    num_tx_slots = channel.shape[2]
    for i in range(num_tx_slots):
        axs = [fig.add_subplot(2,num_tx_slots,i+1),fig.add_subplot(2,num_tx_slots,i+num_tx_slots+1)]
        # plot magnitude
        axs[0].set_xlabel("Subcarrier indices")
        axs[0].set_ylabel("Magnitude (dB)")
        for ii in range(channel.shape[1]):
            axs[0].plot(20*np.log10(np.abs(channel[:, ii, i])),
                label=f"Rx Ant {ii}")
        axs[0].legend()

        # plot phase
        axs[1].set_xlabel("Subcarrier indices")
        axs[1].set_ylabel("Phase (deg)")
        for ii in range(channel.shape[1]):
            axs[1].plot(np.unwrap(np.angle(channel[:, ii, i]))*180/np.pi,
                label=f"Rx Ant {ii}")
        axs[1].legend()

    fig.set_tight_layout(True)
    
    canvas.draw()  # draw the canvas, cache the renderer
    
    width, height = fig.get_size_inches() * fig.get_dpi()
    image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape((int(height), int(width), 3))
    return image

