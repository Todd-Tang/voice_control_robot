import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import fftconvolve

import pyroomacoustics as pra
from pyroomacoustics.doa import circ_dist

import rclpy
from rclpy.node import Node

class DOA(Node):
    def __init__(self):
        super().__init__('DOA_Node')
        self.create_timer(0.5, self.timer_callback)
        self.x = 1.0
        self.y = 1.0
        self.sound_x = 1.0
        self.sound_y = 8.5
        self.sound_speed = 1.0
        self.estimated_angle_radian = 0.0
        self.estimated_angle_degree = 0.0

        


    def timer_callback(self):
        
        location = np.r_[self.sound_x, self.sound_y]
        self.angle(location)
        if self.sound_x > 5.0 or self.sound_x < 1.0:
            self.sound_speed *= -1
            
        self.sound_x += self.sound_speed * 0.3

        self.x += 0.5 * np.cos(self.estimated_angle_radian)
        self.y += 0.5 * np.sin(self.estimated_angle_radian)


    def angle(self,location):
    ######
    # We define a meaningful distance measure on the circle

    # Location of original source
        #azimuth = 90. / 180. * np.pi
        distance = 7.  # meters

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

        corners = np.array([[0,0], [0,10], [6,10], [6,8], [4,8], [4,4], [5,4], [5,0]]).T  # [x,y]
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

        fig, ax = aroom.plot()
        plt.get_current_fig_manager().window.wm_geometry("+3000+50")

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
            algo_names = sorted(pra.doa.algorithms.keys())

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
            self.estimated_angle_degree = self.estimated_angle_radian / np.pi * 180.0
            self.get_logger().info(str(self.estimated_angle_degree))

        except IndexError:
            self.get_logger().info("Object lost, keep going in the previous direction.")

        end_x=mic[0]+7*np.cos(self.estimated_angle_radian )
        end_y=mic[1]+7*np.sin(self.estimated_angle_radian )
        print(mic[0],mic[1])
        print(end_x,end_y)

        ax.plot([mic[0],end_x],[mic[1],end_y],marker='*')

        ax.set_xlim(-1,15)
        ax.set_ylim(-1,15)

        fig.canvas.draw()
        fig.canvas.flush_events()

        

def main(args=None):
    plt.ion()
    rclpy.init(args=args)
    node = DOA()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()