import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from custom_interfaces.srv import SetDistanceAngle
from custom_interfaces.msg import VoiceCommand
import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

from scipy.signal import fftconvolve

import pyroomacoustics as pra
from pyroomacoustics.doa import circ_dist

import std_msgs.msg

class DOA(Node):

    def __init__(self):
        super().__init__('DOA_Node')
        plt.ion()
        self.future = None
        self.distance = 1.0
        self.fig, self.ax = plt.subplots()
        self.come = False
        self.enable_doa = False
        self.sound_end_x = 0.0
        self.sound_end_y = 0.0
        self.set_fig_window_position(self.fig,3000,50)  
        self.create_timer(0.5, self.timer_callback)

        self.voice_command_subscription = self.create_subscription(
            VoiceCommand,
            '/voice_command',
            self.voice_command_callback,
            10)
        
        self.clicked_point_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        
        # Subscription to amcl_pose to get robot's location
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10)

        # Service client for set_distance_angle
        self.client = self.create_client(SetDistanceAngle, 'set_distance_angle')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for "set_distance_angle" service to become available...')
        
        self.x = 0.0
        self.y = 0.0
        self.sound_x = 1.0
        self.sound_y = 1.0
        self.estimated_angle_radian = 0.0
        self.estimated_angle_degree = 0.0


    def amcl_pose_callback(self, msg):
        # Update robot's location
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def voice_command_callback(self, msg):
        if msg.command == 'come':
            if not self.come:  # Check if the previous call is not in progress
                self.come = True
                self.enable_doa = True
                self.distance = msg.number
                self.get_logger().info("Received 'come' command, setting trigger.")
            else:
                self.get_logger().info("Previous call still in progress.")

        elif msg.command == 'right':
            if not self.come:  # Check if the previous call is not in progress
                self.come = True
                self.enable_doa = False
                self.distance = msg.number
                self.estimated_angle_degree = 0.0  
                self.get_logger().info("Received 'right' command, setting trigger.")
            else:
                self.get_logger().info("Previous 'right' call still in progress.")

        elif msg.command == 'left':
            if not self.come:  # Check if the previous call is not in progress
                self.come = True
                self.enable_doa = False
                self.distance = msg.number
                self.estimated_angle_degree = 180.0  
                self.get_logger().info("Received 'left' command, setting trigger.")
            else:
                self.get_logger().info("Previous 'left' call still in progress.")

        elif msg.command == 'up':
            if not self.come:  # Check if the previous call is not in progress
                self.come = True
                self.enable_doa = False
                self.distance = msg.number
                self.estimated_angle_degree = 90.0 
                self.get_logger().info("Received 'up' command, setting trigger.")
            else:
                self.get_logger().info("Previous 'up' call still in progress.")

        elif msg.command == 'down':
            if not self.come:  # Check if the previous call is not in progress
                self.come = True
                self.enable_doa = False
                self.distance = msg.number
                self.estimated_angle_degree = 270.0  
                self.get_logger().info("Received 'down' command, setting trigger.")
            else:
                self.get_logger().info("Previous 'down' call still in progress.")

        elif msg.command == 'stop':
            if not self.come:  # Check if the previous call is not in progress
                self.come = True
                self.enable_doa = False
                self.distance = 0.0
                self.estimated_angle_degree = 0.0  # Set angle to 0 for forward command
                self.get_logger().info("Received 'stop' command, setting trigger.")
            else:
                self.get_logger().info("Previous 'stop' call still in progress.")

    def clicked_point_callback(self, msg):
        self.sound_x = msg.point.x
        self.sound_y = msg.point.y

    def timer_callback(self):
        
        location = np.r_[self.sound_x, self.sound_y]
        
        self.angle(location)
        if self.come:
            if self.future is None or self.future.done():
                self.get_logger().info(str(self.come))
                # location = np.r_[self.sound_x, self.sound_y]
                # self.angle(location)
                req = SetDistanceAngle.Request()
                req.distance = self.distance
                req.angle = self.estimated_angle_degree
                self.future = self.client.call_async(req)
                self.come = False
                self.get_logger().info("Service called with distance: " + str(self.distance))
            else:
                self.get_logger().info("Waiting for previous service call to complete.")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
            
    def set_fig_window_position(self, fig, x, y):

        backend = plt.get_backend()
        if backend == 'TkAgg':
            fig.canvas.manager.window.wm_geometry(f"+{x}+{y}")
        elif backend == 'Qt5Agg':
            fig.canvas.manager.window.move(x, y)

    def angle(self,location):
    ######
    # We define a meaningful distance measure on the circle

    # Location of original source
        #azimuth = 90. / 180. * np.pi
        distance = 2.  # meters

        #dim = 2  # dimensions (2 or 3)
        #room_dim = np.r_[10.0, 10.0]

        # Use AnechoicRoom or ShoeBox implementation. The results are equivalent because max_order=0 for both.
        # The plots change a little because in one case there are no walls.
        #use_anechoic_class = True

        #print("============ Using anechoic: {} ==================".format(anechoic))

        #######################
        # algorithms parameters
        SNR = 0.0  # signal-to-noise ratio
        c = 343.0  # speed of sound
        fs = 16000  # sampling frequency
        nfft = 256  # FFT size
        freq_bins = np.arange(5, 60)  # FFT bins to use for estimation

        # compute the noise variance
        sigma2 = 10 ** (-SNR / 10) / (4.0 * np.pi * distance) ** 2

        corners = np.array([[-1,0.5], [0.2,-1.5], [0.6,-1.5], [0.9,-2], [3.0,-2], [3.4,-1.5], [3.8,-1.5], [4.66,0],
                            [4.35, 0.5],[4.68,1.05],[3.8,2.5],[3.4,2.5],[3.1,3],[0.9,3],[0.6,2.5],[0.2,2.5]]).T  # [x,y]
        aroom = pra.Room.from_corners(corners, fs=fs, max_order=0, sigma2_awgn=sigma2)
        mic = [self.x, self.y]

        # add the source
        echo = pra.circular_2D_array(center=mic, M=6, phi0=0, radius=0.2)
        echo = np.concatenate((echo, np.array(mic, ndmin=2).T), axis=1)
        aroom.add_microphone_array(pra.MicrophoneArray(echo, aroom.fs))

        # Add sources of 1 second duration
        rng = np.random.RandomState(23)
        duration_samples = int(fs)

        #source_location = mic + distance * np.r_[np.cos(azimuth), np.sin(azimuth)]
        source_location = location
        source_signal = rng.randn(duration_samples)
        aroom.add_source(source_location, signal=source_signal)

        self.ax.clear()
        temp_fig, temp_ax = aroom.plot() 
        self.set_fig_window_position(temp_fig,0,0)  


        for line in temp_ax.lines:
            self.ax.plot(*line.get_data(), marker=line.get_marker(), color=line.get_color())
        for scatter in temp_ax.collections:
            # Assuming scatter plot is created using `ax.scatter()`
            offsets = scatter.get_offsets()
            color = scatter.get_edgecolor()
            self.ax.scatter(offsets[:, 0], offsets[:, 1], color=color)
        for patch_collection in temp_ax.collections:
            for path in patch_collection.get_paths():
                polygon = Polygon(path.vertices, closed=True, facecolor='none', edgecolor='black')
                self.ax.add_patch(polygon)

        plt.close(temp_fig)
        

        #plt.get_current_fig_manager().window.wm_geometry("+3000+50")

        # run the simulation
        try:
            aroom.simulate()

            ################################
            # Compute the STFT frames needed
            X = np.array(
                [
                    pra.transform.stft.analysis(signal, nfft, nfft // 2).T
                    for signal in aroom.mic_array.signals
                ]
            )

            ##############################################
            # Now we can test all the algorithms available
            #algo_names = sorted(pra.doa.algorithms.keys())
            algo_names = [#'SRP', 
                          'MUSIC', 
                          #'FRIDA', 
                          #'TOPS'
                          ]

            angle_sum = 0.0
            for n, algo_name in enumerate(algo_names):
                # Construct the new DOA object
                # the max_four parameter is necessary for FRIDA only
                doa = pra.doa.algorithms[algo_name](echo, fs, nfft, c=c, max_four=4)

                # this call here perform localization on the frames in X
                doa.locate_sources(X, freq_bins=freq_bins)

                #doa.polar_plt_dirac()
                #plt.title(algo_name)

                # doa.azimuth_recon contains the reconstructed location of the source
                #print(algo_name)
                #print("  Recovered azimuth:", doa.azimuth_recon / np.pi * 180.0, "degrees")
                #print("  Error:", circ_dist(azimuth, doa.azimuth_recon) / np.pi * 180.0, "degrees")
                angle_sum += doa.azimuth_recon

            #plt.show()
            #print("Estimated angle: ", angle_sum[0]/(n+1) / np.pi * 180.0, "degrees")
            self.estimated_angle_radian = angle_sum[0]/(n+1)

            if self.enable_doa:
                self.estimated_angle_degree = self.estimated_angle_radian / np.pi * 180.0
                self.get_logger().info(str(self.estimated_angle_degree))
                

        except IndexError:
            self.get_logger().info("Object lost, keep going in the previous direction.")

        
        # print(mic[0],mic[1])
        # print(end_x,end_y)
        self.sound_end_x=mic[0]+7*np.cos(self.estimated_angle_radian)
        self.sound_end_y=mic[1]+7*np.sin(self.estimated_angle_radian)

        dir_end_x=mic[0]+3*np.cos(np.radians(self.estimated_angle_degree))
        dir_end_y=mic[1]+3*np.sin(np.radians(self.estimated_angle_degree))

        self.ax.plot([self.x,self.sound_end_x],[self.y,self.sound_end_y],marker='*',color='blue')
        self.ax.plot([self.x,dir_end_x],[self.y,dir_end_y],marker='*',color='red')

        self.ax.set_xlim(-1.5,5)
        self.ax.set_ylim(-3,3.5)

        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = DOA()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
