# !/usr/bin/env python3

from scipy.signal import butter
import numpy as np

class ButterworthFilter:

    def __init__(self, sampling_frequency, cutoff_frequency, order, btype, analog): 
        """
        Filter parameters

        - cutoff_frequency = Cut-off frequency of the low-pass filter (Hz)
        - order = Order of the Butterworth filter
        - b, a : ndarray, ndarray. Numerator (`b`) and denominator (`a`) polynomials of the IIR filter.
        """              
        # cutoff_frequency = 25
        # sampling_frequency = 100
        self.filtered_data_x = []
        self.filtered_data_y = []
        self.sample_data_x = []
        self.sample_data_y = []
        self.order = order

        nyquist = 0.5 * sampling_frequency
        normal_cutoff = cutoff_frequency / nyquist

        self.b, self.a = butter(order, normal_cutoff, btype, analog)
        # print('x_s: ', self.b, 'y_s: ', self.a)
        # self._xs = deque([0] * len(self.b), maxlen=len(self.b))
        # self._ys = deque([0] * (len(self.a) - 1), maxlen=len(self.a)-1)
        self.iter = 0

        self.data_buffer_x =  np.zeros(3)
        self.data_buffer_y = np.zeros(3)
        self.filtered_buffer_x = np.zeros(2)
        self.filtered_buffer_y = np.zeros(2)
        # filtered_data = np.zeros(len(self.sample_data))

    # def reset()
        
    def update(self, new_raw_data):

        self.sample_data_x.append(new_raw_data)
        if self.iter <= 1:
            self.data_buffer_x[0] = new_raw_data
            self.filtered_buffer_x[0] = new_raw_data
            self.filtered_data_x.append(self.filtered_buffer_x[0]) 
            self.data_buffer_x[2] = self.data_buffer_x[1]
            self.data_buffer_x[1] = self.data_buffer_x[0]
            self.filtered_buffer_x[1] =  self.filtered_buffer_x[0]
            self.iter +=1
        else:
            self.data_buffer_x[0] = new_raw_data
            self.filtered_buffer_x[0] = self.b[0]*self.data_buffer_x[0]+self.b[1]*self.data_buffer_x[1]+self.b[2]*self.data_buffer_x[2]-self.a[1]*self.filtered_buffer_x[0]-self.a[2]*self.filtered_buffer_x[1]
            self.filtered_data_x.append(self.filtered_buffer_x[0])
            self.data_buffer_x[2] = self.data_buffer_x[1]
            self.data_buffer_x[1] = self.data_buffer_x[0]
            self.filtered_buffer_x[1] =  self.filtered_buffer_x[0]

        return self.filtered_buffer_x[0]