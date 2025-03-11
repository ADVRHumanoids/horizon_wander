# !/usr/bin/env python3

from scipy import signal
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

        self.b, self.a = signal.butter(order, normal_cutoff, btype, analog)
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
    
class ButterworthWrenches():
    def __init__(self, sampling_frequency, cutoff_frequency, order = 2, btype = 'low', analog = False): 
        """
        Filter parameters

        - cutoff_frequency = Cut-off frequency of the low-pass filter (Hz)
        - order = Order of the Butterworth filter
        - b, a : ndarray, ndarray. Numerator (`b`) and denominator (`a`) polynomials of the IIR filter.
        """              
        self.fs = sampling_frequency #sampling frequency

        self.fc = cutoff_frequency # Cut-off frequency of the filter
        self.w = self.fc / (self.fs / 2) # Normalize the frequency

        self.b, self.a = signal.butter(order, self.w, 'low')

        # self.zi = signal.lfilter_zi(self.b, self.a)

        # z, _ = signal.lfilter(self.b, self.a, self.FX, zi=self.zi*self.FX[0])
        # z2, _ = signal.lfilter(b, a, z, zi=self.zi*z[0])
        # y = signal.filtfilt(b, a, xn)

        self.FX = []
        self.FY = []
        self.FZ = []

        self.TX = []
        self.TY = []
        self.TZ = []

        self.filtered_data_x = []
        self.filtered_data_y = []
        self.sample_data_x = []
        self.sample_data_y = []
    
    def update(self, data):

        fx = data[0]
        fy = data[1]
        fz = data[2]
        tx = data[3]
        ty = data[4]
        tz = data[5]

        self.FX.append(fx)
        # if len(self.FX) > 500 : self.FX = self.FX[1:]
        self.FY.append(fy)
        # if len(self.FY) > 500 : self.FY = self.FY[1:]
        self.FZ.append(fz)
        # if len(self.FZ) > 500 : self.FZ = self.FZ[1:]

        self.TX.append(tx)
        # if len(self.TX) > 500 : self.TX = self.TX[1:]
        self.TY.append(ty)
        # if len(self.TY) > 500 : self.TY = self.TY[1:]
        self.TZ.append(tz)
        # if len(self.TZ) > 500 : self.TZ = self.TZ[1:]

        fx_low = signal.lfilter(self.b, self.a, self.FX) #Forward filter
        fy_low = signal.lfilter(self.b, self.a, self.FY)
        fz_low = signal.lfilter(self.b, self.a, self.FZ)

        tx_low = signal.lfilter(self.b, self.a, self.TX)
        ty_low = signal.lfilter(self.b, self.a, self.TY)
        tz_low = signal.lfilter(self.b, self.a, self.TZ)

        # fx_low = signal.filtfilt(self.b, self.a, self.FX) #Forward filter
        # fy_low = signal.filtfilt(self.b, self.a, self.FY)
        # fz_low = signal.filtfilt(self.b, self.a, self.FZ)

        # tx_low = signal.filtfilt(self.b, self.a, self.TX)
        # ty_low = signal.filtfilt(self.b, self.a, self.TY)
        # tz_low = signal.filtfilt(self.b, self.a, self.TZ)


        filtered_wrenches = np.array([fx_low[-1], fy_low[-1], fz_low[-1], tx_low[-1], ty_low[-1], tz_low[-1]])

        return filtered_wrenches