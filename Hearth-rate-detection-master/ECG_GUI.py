import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import threading
import time
from collections import deque
from PT import *


class ECG_GUI():
    def __init__(self,Fs,win_len,serialReader,panTompkins):
        #Zajebi
        #IAO
        self.fig, (self.ax1,self.ax2) = plt.subplots(2,1)
        self.fig_lines1={
            'signal': self.ax1.plot([],[],label = 'Signal')[0],
            'peaks' : self.ax1.plot([],[],'xr',label = 'Peaks')[0],
           
        }
        self.fig_lines2 = {
            'signal': self.ax2.plot([],[],label = 'Signal')[0],
            'peaks' : self.ax2.plot([],[],'xr',label = 'Peaks')[0],
            'signal_level':self.ax2.plot([],[],label = 'Signal level')[0],
            'noise_level':self.ax2.plot([],[],label = 'Noise level')[0],
            'threshold':self.ax2.plot([],[],label = 'Threshold level')[0],
        }
        self.y_data = None
        self.x_data = None
        self.Fs = Fs
        self.win_len = win_len
        self.data_queue = deque()
        self.serialReader = serialReader
        self.pt = panTompkins

        self.ax1.set_title('Original signal');self.ax1.set_ylabel('Voltage[V]')
        self.ax2.set_title('Filtered signal');self.ax2.set_ylabel('Voltage[V]');self.ax2.set_xlabel('t[s]')
        plt.ion()
        plt.show()

        #Prettify representation
        pos = self.ax2.get_position()
        self.ax2.set_position([pos.x0, pos.y0+pos.height*0.1, pos.width, pos.height])
        self.ax2.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4),ncol=5,fancybox=True, shadow=True)

        pos = self.ax1.get_position()
        self.ax1.set_position([pos.x0, pos.y0 + pos.height*0.1, pos.width, pos.height])

    def _collectData(self):
        while True:
            data = serialReader.read_data(read_time=2)
            self.data_queue.append(data)
    def _processData(self,data):
        self.x_data,self.y_data = pt.find_peaks(data,verbose=False)



    def _getData(self,y_data,x_data):
        self.y_data = y_data
        self.x_data = x_data

    def _updatePlot(self,ax_num=1):
        #Prepare current axis
        fig_lines = self.fig_lines1
        ax = self.ax1
        x_data = self.x_data['original']
        y_data = self.y_data['original']
        ax.set_title(f"Original signal, BPM:{pt.BPM:.0f}")
        if ax_num == 2:
            fig_lines = self.fig_lines2
            ax = self.ax2     
            x_data = self.x_data['filtered']
            y_data = self.y_data['filtered']

        #Get data
        y_values = []
        x_values = []
        #Append data on graph
        for k, v in fig_lines.items():
            y_values.append(v.get_ydata())
            x_values.append(v.get_xdata())      
        #Update y_values with new data
        for i, (k, v) in enumerate(y_data.items()):
            y_values[i] = np.append(y_values[i],v)
        #Update x_values with new data
        for i, (k, v) in enumerate(x_data.items()):
            x_values[i] = np.append(x_values[i],v)
        
        #Bound x and y values given win_length
        if len(y_values[0])>self.win_len*self.Fs:
                y_values[0] = y_values[0][-self.win_len*self.Fs:]
                x_values[0] = x_values[0][-self.win_len*self.Fs:]

        for i in range(1,len(x_values)):
            if x_values[i][0]<x_values[0][0]:
                try:
                    a = np.where(x_values[i]>x_values[0][0])
                    cut_off = np.where(x_values[i]>x_values[0][0])[0][0]
                    x_values[i] = x_values[i][cut_off:]
                    y_values[i] = y_values[i][cut_off:]
                except IndexError:
                    pass

        #Plot
        for i,(k, v) in enumerate(fig_lines.items()):
            v.set_ydata(y_values[i])
            v.set_xdata(x_values[i])
        ax.relim()
        ax.autoscale_view()

    
    def live_plot(self):
        data_thread = threading.Thread(target=self._collectData)
        data_thread.daemon = True
        data_thread.start()
        while True:
            if self.data_queue:
                raw_data = self.data_queue.popleft()
                self._processData(raw_data)
                self._updatePlot(ax_num=1)
                self._updatePlot(ax_num=2)
                plt.draw()
                plt.pause(0.01)
            else:
                time.sleep(0.1)



        
        

#%%

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
pt.init_R_peaks(init_data)
plt.show()

#Initialize GUI
ecgGUI = ECG_GUI(Fs=360,win_len=3,serialReader=serialReader,panTompkins=pt)
#Start detection
print("Detection starts")
ecgGUI.live_plot()


