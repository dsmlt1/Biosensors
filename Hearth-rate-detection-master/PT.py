import serial
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans
from scipy import signal


class Buffer():
    def __init__(self,size):
        self.size = size
        self.iter = -1
        self.data = [-1]*self.size

    def get_elem(self):
        return self.data[self.iter]
    
    def put_next(self,x):
        self.iter = (self.iter+1)%self.size
        self.data[self.iter] = x

    def get_data(self):
        return self.data
    
    def __str__(self):
        return str(self.data)

class PanTompkins():
    def __init__(self,Fs):
        self.Fs = Fs
        self.peaks_buffer= Buffer(50)
        self.peaks_idx = []
        self.sig_idx = 0
        #Two RR values closer that pause_duration won't be detected
        self.pause_duration=None
        self.monitor_index=0
        self.   ma_window = 50
        self.old_signal = np.zeros(self.ma_window)
        self.BPM = -1
        

    def initialize(self, sig,verbose=True):
        """
        Initialization of threshold values for the signal
        and noise
        """
        sig = self.bandpass_filter(sig)
        peaks = signal.find_peaks(sig)[0]   
        # fig = plt.figure()
        # plt.plot(sig);plt.xlabel('samples[n]');plt.ylabel('Voltage[V]')
        # plt.plot(peaks,sig[peaks],'rx')
        # plt.show();plt.title("Find peaks")

        #Clusterization
        kmeans = KMeans(n_clusters=2,n_init=10)
        kmeans.fit(np.array(sig[peaks]).reshape(-1,1))
        #Classify
        Z = kmeans.predict(np.c_[sig[peaks]])
        signal_peaks = sig[peaks[Z==1]]
        noise_peaks = np.abs(sig[peaks[Z==0]]) ##ISPADA I NEGATIVNO?


        # self.sig_level = np.max(sig[peaks])
        # self.noise_level= np.min(sig[peaks])
        self.sig_level = np.mean(signal_peaks)
        self.noise_level= np.mean(noise_peaks)
        #Assign correct threshold if clusterization didn't
        if self.sig_level<self.noise_level:
            t = self.sig_level
            self.sig_level = self.noise_level
            self.noise_level = t
        self.thresh = self.noise_level + 0.4*(self.sig_level-self.noise_level)
        print(f"Threshold initialized:{self.thresh:.2f}")
        print(f"Noise level initialized:{self.noise_level:.2f}")
        print(f"Signal level initialized:{self.sig_level:.2f}")


        #Visualize
        if verbose==True:
            ##Clusterization
             # Plot the decision boundary. For that, we will assign a color to each
            x_min, x_max = 0, len(sig)
            y_min, y_max = sig.min(), sig.max() 
            xx, yy = np.meshgrid(np.arange(x_min, x_max, 10), np.arange(y_min, y_max, 0.01))
            Z = kmeans.predict(np.c_[yy.ravel()])
            # Put the result into a color plot
            Z = Z.reshape(xx.shape)
            plt.figure(1)
            plt.clf()
            plt.imshow(
                Z,
                interpolation="nearest",
                extent=(xx.min(), xx.max(), yy.min(), yy.max()),
                aspect="auto",
                origin="lower",
                alpha = 0.5
            )
            ##Signal
            plt.plot(sig,label='Signal')
            plt.plot(peaks,sig[peaks],'xr',label='peaks')
            plt.title("Threshold initialization");plt.xlabel('n[samples]');plt.ylabel('Voltage[V]')
            plt.axhline(self.thresh,label ='Signal threshold',color = 'g' )
            plt.axhline(self.noise_level,label='Noise level',color='r')
            plt.axhline(self.sig_level,label='Signal level',color='b')

            plt.legend()
            plt.autoscale()
           
        
    def init_R_peaks(self,sig,verbose=True):
        """
        Initialization of RR peak value and pause duration
        We find first two valid peaks and take distance between them
        as RR peak value
        """
        #Preprocessing
        sig = self.bandpass_filter(sig)
        ma_sig = sig
        #Find peaks
        peaks = signal.find_peaks(ma_sig)[0]
        valid_peaks = []
        for i, peak in enumerate(peaks):
            #Refacotry period of 200ms
            if len(valid_peaks)>0:
                if (peak-valid_peaks[-1])<0.2*self.Fs:
                    continue
            if ma_sig[peak]>self.thresh:
                valid_peaks.append(peak)
            if len(valid_peaks)==2:
                break
        
        sig_peaks = valid_peaks
        
        #Calculate RR value
        self.RR_1 = sig_peaks[-1]-sig_peaks[0]
        self.RR_2 = self.RR_1
        self.RR_1s = np.ones(8)*self.RR_2
        self.RR_2s = np.ones(8)*self.RR_1
        
        #Initialize pause duration 
        self.pause_duration = self.RR_1*0.3
        print(f"RR values are initialized: {self.RR_1:.0f}samp({self.RR_1/self.Fs:.2f}s)")
        #Visualize
        if verbose:
            plt.figure()
            plt.plot(sig)
            plt.plot(sig_peaks,sig[sig_peaks],'xr')
            plt.title("RR_peaks initialization")
            plt.xlabel('n[samples]');plt.ylabel('Voltage[V]')

    def check_peak(self,sig,monitor=True):
        #Reset monitor_dict
        self.monitor_dict =  {
                'thresh':[],
                'sig_level': [],
                'noise_level': [],
                'time_stamps': [],
                'RR1_intervals': [],
                'RR2_intervals': []
                }
        self.monitor_index=0 #For representation of sig
        #Find peaks
        peaks = signal.find_peaks(sig)[0]
        # fig = plt.figure()
        # plt.plot(sig)
        # plt.plot(peaks,sig[peaks],'xr')
        # plt.title("find_peaks function output")
        #Initialize
        
        detect_index = 0
        rr1_index= 0
        for i,peak in enumerate(peaks):
            #refractory period, no 2 beats can be this close
            if (peak+self.sig_idx-self.peaks_buffer.get_elem())<self.pause_duration:
                continue

            elif (peak - self.peaks_buffer.get_elem())>1.66*self.RR_1:
                #Peak is missdetected, searchback using lower threshold
                best_candidate = np.argmax(sig[peaks[detect_index:i]])
                self.peaks_idx.append(self.peaks_buffer.iter)
                self.peaks_buffer.put_next(peaks[best_candidate+detect_index]+self.sig_idx)
                self.sig_level = 0.3*sig[peak]+0.7*self.sig_level
                detect_index=i

            elif sig[peak]>self.thresh:            
                #Update signal level
                self.sig_level = 0.3*sig[peak]+0.7*self.sig_level 

                #Update RR intervals         
                self.RR_1s[rr1_index]=peak+self.sig_idx-self.peaks_buffer.get_elem()
                rr1_index = rr1_index%8 #circular buffer
                self.RR_1 = np.mean(self.RR_1s)
                self.BPM = 60/(self.RR_1/self.Fs)
                self.pause_duration = self.RR_1*0.5
                detect_index=i
                self.peaks_buffer.put_next(peak+self.sig_idx)
                self.peaks_idx.append(self.peaks_buffer.iter)

                if monitor:
                    self.monitor_dict['RR1_intervals'].append(self.RR_1)
                    self.monitor_dict['RR2_intervals'].append(self.RR_2)
        
            else:
                #Update noise level
                self.noise_level = 0.3*np.abs(sig[peak])+0.7*self.noise_level
            
            #Update threshold
            self.thresh = self.noise_level + 0.4*(self.sig_level-self.noise_level)

            if monitor:
                self.monitor_dict[f'thresh'].append(self.thresh)
                self.monitor_dict[f'sig_level'].append(self.sig_level)
                self.monitor_dict[f'noise_level'].append(self.noise_level)
                self.monitor_dict[f'time_stamps'].append(peak)
                self.monitor_index+=1

    def bandpass_filter(self,ecg_sig, verbose=False):
         ecg_sig = np.concatenate((self.old_signal,ecg_sig))
         signal = np.convolve(ecg_sig, np.ones(self.ma_window) / self.ma_window, mode='same')
         signal = signal-np.mean(signal)
         self.old_signal[:] = ecg_sig[-self.ma_window:]
         if verbose:
             fig, (ax1,ax2) = plt.subplots(2,1)
             ax1.set_title("Original signal")
             ax1.plot(ecg_sig,label='Signal');ax1.set_ylabel('Voltage[V]')
             ax2.plot(signal)
             ax2.set_xlabel('n[samples]');ax2.set_ylabel('Voltage[V]');ax2.set_title("Filtered signal")
            

         return signal[self.ma_window:]


    def derivative(self,sig):
        filtered = np.zeros(len(sig))
        for i in range(len(sig)-2):
            if i>=2:
                filtered[i] = (-sig[i-2]-2*sig[i-1]+2*sig[i+1]+sig[i+2])/8
        return filtered
    
    def ma_filter(self,signal, threshold=0):
        """
        Apply a moving average filter to a signal.
        
        Parameters:
        signal (array-like): The input signal to be filtered.
        window_length (int): The length of the moving average window.
        threshold (float, optional): The threshold below which to set filtered signal values to zero. Default is 0.
        
        Returns:
        np.ndarray: The filtered signal.
        """
        
        signal = np.asarray(signal)    
        # Apply moving average filter
        filtered_signal = np.convolve(signal, np.ones(self.ma_window) / self.ma_window, mode='same')
        # Apply threshold if specified
        if threshold != 0:
            filtered_signal[filtered_signal < threshold] = 0
        

        return filtered_signal

    def localize_peak(self,sig):
        
        latency = self.ma_window #How much is MA signal delay
        #np.ceil(Fs*0.).astype(int) #30 ms localization
        peaks = []
        peaks_valid = np.array(self.peaks_buffer.get_data())[self.peaks_idx]
        # peaks_valid_supressed=[peaks_valid[0]]
        # for i in range(1,len(peaks_valid)):
        #     if peaks_valid[i]-peaks_valid[i-1]>self.pause_duration:
        #         peaks_valid_supressed.append(peaks_valid[i])
        # peaks_valid = peaks_valid_supressed

        for peak in peaks_valid:
            peak = peak - self.sig_idx
            lower = max(0,peak-latency)
            #upper = min(peak+latency,len(sig))
            upper = peak
            best = np.argmax(sig[lower:upper])
            peaks.append(lower+best+self.sig_idx)
        return peaks
 
    def find_peaks(self,sig,verbose=False,animation=False):
        self.peaks_idx=[] 
        orig_sig = np.array(sig)   
        sig = self.bandpass_filter(sig)
        #deriv = self.derivative(sig)
        #square_sig = deriv**2
        #ma_sig = self.ma_filter(sig)
        ma_sig = sig
        self.check_peak(ma_sig)
        peaks_valid = np.array(self.localize_peak(orig_sig))
        peaks_valid1 =  np.array(self.peaks_buffer.get_data())[self.peaks_idx]
        peaks_valid = peaks_valid - self.sig_idx
        peaks_valid1 = peaks_valid1 - self.sig_idx

        #time axis
        t = np.linspace(self.sig_idx/self.Fs, (self.sig_idx+len(sig))/self.Fs,len(sig))
        peaks_masig_locations = (self.sig_idx+np.array(peaks_valid1))/self.Fs
        peaks_orig_locations = (self.sig_idx+np.array(peaks_valid))/self.Fs
        monitor_t = (self.sig_idx+np.array(self.monitor_dict['time_stamps'][-self.monitor_index:]))/self.Fs
        if verbose:
            fig = plt.figure()
            plt.plot(t,ma_sig)
            plt.plot(peaks_masig_locations,ma_sig[peaks_valid1],'xr')
            plt.plot(monitor_t,self.monitor_dict['thresh'][-self.monitor_index:],label='Threshold1')
            plt.plot(monitor_t,self.monitor_dict['sig_level'][-self.monitor_index:],label='Signal level1')
            plt.plot(monitor_t,self.monitor_dict['noise_level'][-self.monitor_index:],label='Noise level1')
            plt.title("Filtered signal and thresholds")
            plt.ylabel('Amplitude[a.u]')
            plt.xlabel('t[s]')
            plt.legend()

            fig = plt.figure()
            plt.plot(t, orig_sig)
            plt.plot(peaks_orig_locations,orig_sig[peaks_valid],'xr')
            plt.title("Original signal with peaks")
            plt.ylabel('Amplitude[a.u]')
            plt.xlabel('t[s]')
            
        y_data = {
            'original':
                {'signal':orig_sig,
                'peaks':orig_sig[peaks_valid],
                },
            'filtered':
                {
                'signal' : ma_sig,
                'peaks':ma_sig[peaks_valid1],
                'signal_level':self.monitor_dict['sig_level'],
                'noise_level':self.monitor_dict['noise_level'],
                'threshold':self.monitor_dict['thresh'],    
                }         
        }
        x_data = {
            'original':{
                'signal': t,
                'peaks': peaks_orig_locations,
            },
            'filtered':{
                'signal': t,
                'peaks':peaks_masig_locations,
                'signal_level' : monitor_t,
                'noise_level' : monitor_t,
                'threshold' : monitor_t, 
            }
        }



        self.sig_idx+=len(sig) #Save information on real indices
        #Return data for plotting
        return(x_data,y_data)

class SerialReader():
    def __init__(self,Fs,serial):
        self.Fs = Fs
        self.serial = serial

    def read_data(self,read_time):
        k = 0
        samples = []
        data_size = read_time*self.Fs
        while (k < data_size):
            if self.serial.inWaiting() > 0:
                m = self.serial.readline()
                try:
                    r = float(m)
                    samples.append(r)
                    k = k + 1
                except:
                    pass
        return samples



if __name__=='__main__':
    #Communication initialization
    s = serial.Serial('com10', 9600) # podesite odgovarajuci COM port
    serialReader = SerialReader(Fs=360,serial=s)
    pt =  PanTompkins(Fs=360)

    #Initialize PanTompkins params
    print("Initialize thresholds")
    init_data = serialReader.read_data(read_time=3)
    plt.show()
    pt.initialize(init_data)
    print("Initialize RR values")
    plt.show()
    #init_data = serialReader.read_data(read_time=2)
    pt.init_R_peaks(init_data)
    plt.show()
    #Start detection
    print("Detection starts")
    while True:  
        data = serialReader.read_data(read_time=2)
        peaks = pt.find_peaks(data,verbose=True)
        plt.show()


    

