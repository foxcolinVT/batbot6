#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 12:33:23 2020
@author: devan
Updated: July 7, 2022
By: Colin Fox, Hayley Wisman
"""

import os
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import numpy as np
from pyqtgraph import ColorBarItem
from scipy.signal import spectrogram, butter, lfilter
from scipy import signal
import time
#import colormaps as cm  # using online module colormaps
from enum import Enum
from qtrangeslider import QRangeSlider  # replacing Rangeslider
from matplotlib import cm
import requests

os.environ['DISPLAY'] = ':0'  # fixes display error with launch on startup


class SizePolicy(Enum):
    """
    Setting sizing for resizing widgets
    """

    FIX = QtWidgets.QSizePolicy.Fixed
    MIN = QtWidgets.QSizePolicy.Minimum
    MAX = QtWidgets.QSizePolicy.Maximum
    PREF = QtWidgets.QSizePolicy.Preferred
    EXPAND = QtWidgets.QSizePolicy.Expanding
    MINEXPAND = QtWidgets.QSizePolicy.MinimumExpanding


class Align(Enum):
    """
    Setting alignment for widgets
    """
    CENTER = QtCore.Qt.AlignCenter
    LEFT = QtCore.Qt.AlignLeft
    RIGHT = QtCore.Qt.AlignRight


class Window(QtWidgets.QWidget):

    def __init__(self):
        """
        Initializing the Window and graphical aspects
        """

        super().__init__()

        self.setWindowIcon(QtGui.QIcon('VT_BIST_LOGO.ico'))
        self.setWindowTitle('BatBot')

        # CONSTANTS
        self.width = 800        # width of the window (pixels)
        self.height = 430       # height of the window (pixels)
        self.plotWidth = 375    # width of each plot (pixels)
        self.numDataPoints = int(10e3)      # number of data points per file
        self.samplingFrequency = int(400e3)    # sampling frequency (Hz)
        self.beginTime = 0      # start time of sampling
        self.stopTime = 0.025   # stop time of sampling (seconds)
        self.maxFreq = 200e3    # maximum recorded frequency (Hz)

        # STRINGS FOR LABELS
        self.nString = 'N: '                     # elapsed number of iterations
        self.tString = 't: '                     # elapsed time since recording start
        self.samplingFrequencyString = 'Fs: '    # sampling frequency
        self.fpsString = 'fps: '                 # frames per second of the displayed plots
        self.nMaxString = 'Nmax: '               # maximum number of echoes

        # VARIABLES
        #self.file = "20181111_121335587.txt"    # IMPORTANT: initial displayed data
        response = requests.get("http://127.0.0.1:8080")      # Request file from web server
        with open("newfile.txt", "w") as file:                # Write contents of web server response to text file
            file.write(response.text)
        self.file = 'newfile.txt'
        self.fps = 0.0                          # frames per second
        self.tPassed = 0                        # time elapsed
        self.timeArray = np.linspace(self.beginTime, self.stopTime, self.numDataPoints)  # array of sample time points
        self.n = 0                              # TODO: figure out what self.n is; used in for loop in run()
        self.sigWindow = 'blackman'             # default signal window
        self.lowcut = 20e3                      # low end of passed frequency range
        self.highcut = 100e3                    # high end of passed frequency range
        self.nperseg = 256                      # number of freq points per time segment
        self.noverlap = self.nperseg//5         # overlap of time segments
        self.nfft = self.nperseg                # not really sure
        self.dBCutoff = 70
        self.truncated_amp_val = -100  # The value truncated amplitude values are changed to in the build_sg function

        self.lCbar = None
        self.rCbar = None
        self.minAmp = None
        self.lImg = pg.ImageItem()
        self.rImg = pg.ImageItem()

        # timer for automatic execution
        # self.timer = QtCore.QTimer()
        # self.timer.setInterval(500) #in ms
        # self.timer.timeout.connect(self.fpsUpdate)
        # self.timer.start()

        # CREATING MAIN GRID LAYOUT
        self.resize(self.width, self.height)
        self.mainLayout = QtWidgets.QGridLayout()
        self.setLayout(self.mainLayout)

        self.create_info_widget()
        self.create_settings()

        self.display_info()  # initializing program on main info screen
        self.update_labels()

        # window starts with spectrogram set to logspace
        self.logspace_button.toggle()
        self.logspace_selected = True

        # SPECTROGRAM SETTINGS MENU
        self.sgBtn.toggle()  # window starts with sgBtn toggled so spectrogram appears

        # Colorbar object initialization
        #self.lCbar = None
        #self.rCbar = None
        #self.minAmp = None
        #self.lImg = pg.ImageItem()
        #self.rImg = pg.ImageItem()


    def create_info_widget(self):
        """
        Creates the main information display
        """

        # groups all the information into one widget
        self.infoWidget = QtWidgets.QWidget()
        self.infoLayout = QtWidgets.QGridLayout()
        self.infoWidget.setLayout(self.infoLayout)

        # create plot layout
        self.view = pg.GraphicsView()  # widget to display plots
        self.plotLayout = pg.GraphicsLayout()
        self.view.setCentralItem(self.plotLayout)

        # add plots
        self.mainLayout.addWidget(self.view)

        # add info widget
        self.mainLayout.addWidget(self.infoWidget, 1, 0, 1, 2)

        # spectrogram radio button
        self.sgBtn = QtWidgets.QRadioButton('Spectrogram')
        self.sgBtn.toggled.connect(self.plotSelection)

        # spectrum radio button
        self.smBtn = QtWidgets.QRadioButton('Spectrum')
        self.smBtn.toggled.connect(self.plotSelection)

        # oscillogram radio button
        self.ogBtn = QtWidgets.QRadioButton('Oscillogram')
        self.ogBtn.toggled.connect(self.plotSelection)

        self.leftPlot = self.plotLayout.addPlot()  # create plots
        self.rightPlot = self.plotLayout.addPlot()
        # self.leftPlot.setEnabled(False)     # disables plot interaction
        # self.rightPlot.setEnabled(False)      # TODO: add button in settings to disable plot interaction?
        self.leftPlot.setFixedWidth(self.plotWidth)
        self.rightPlot.setFixedWidth(self.plotWidth)

        # WIDGETS -------------------------------------------------------
        # button for the settings menu
        self.menuBtn = QtWidgets.QPushButton('Settings')

        # start/stop button
        self.startBtn = QtWidgets.QPushButton('Start/Stop')
        self.startBtn.setFixedWidth(self.plotWidth//4)
        self.startBtn.setStyleSheet("background-color: green")
        self.startBtn.clicked.connect(self.run)
        self.startBtn.setCheckable(True)

        # start/stop pinna deformation button
        self.deformBtn = QtWidgets.QPushButton('Start/Stop Deform')
        self.deformBtn.setFixedWidth(self.plotWidth//3)
        self.deformBtn.setStyleSheet("background-color: green")
        self.deformBtn.clicked.connect(self.run)
        self.deformBtn.setCheckable(True)

        # upload waveform button
        self.uploadBtn = QtWidgets.QPushButton('Upload Waveform')
        self.uploadBtn.setFixedWidth(3*self.plotWidth//10)
        self.uploadBtn.clicked.connect(self.waveformUpload)

        # blank input for number of echoes
        self.iterationInput = QtWidgets.QLineEdit('Infinity')
        self.iterationInput.setSizePolicy(SizePolicy.MIN.value, SizePolicy.MIN.value)

        # LABELS
        self.iterationLabel = QtWidgets.QLabel(self.nString)
        self.durationLabel = QtWidgets.QLabel(self.tString)
        # self.durationLabel.setHorizontalStretch(3)
        self.rateLabel = QtWidgets.QLabel(self.samplingFrequencyString)
        self.fpsLabel = QtWidgets.QLabel(self.fpsString)
        self.nMaxLabel = QtWidgets.QLabel(self.nMaxString)

        # ALIGNMENTS
        left = Align.LEFT.value
        right = Align.RIGHT.value

        # ADDING WIDGETS TO LAYOUT IN PROPER POSITIONS
        # left a gap on column 1, 3, 5
        self.infoLayout.addWidget(self.uploadBtn, 4, 0)
        self.infoLayout.addWidget(self.startBtn, 4, 2)
        self.infoLayout.addWidget(self.deformBtn, 5, 0)
        self.infoLayout.setColumnMinimumWidth(3, 20)
        self.infoLayout.addWidget(self.sgBtn, 3, 4, 1, 1, alignment=left)
        self.infoLayout.addWidget(self.smBtn, 4, 4, 1, 1, alignment=left)
        self.infoLayout.addWidget(self.ogBtn, 5, 4, 1, 1, alignment=left)
        self.infoLayout.setColumnMinimumWidth(5, 20)
        self.infoLayout.addWidget(self.iterationLabel, 3, 6, alignment=left)
        self.infoLayout.addWidget(self.durationLabel, 4, 6)
        self.infoLayout.addWidget(self.rateLabel, 5, 6)
        self.infoLayout.addWidget(self.nMaxLabel, 3, 7, alignment=right)
        self.infoLayout.addWidget(self.iterationInput, 3, 8)
        self.infoLayout.addWidget(self.fpsLabel, 4, 8)
        self.infoLayout.addWidget(self.menuBtn, 5, 8)

        # self.infoLayout.setContentsMargins(5, 0, 5, 5)

    def create_settings(self):
        """
        Creates the settings menus; universal, spectrogram, spectrum, oscillogram
        """

        # UNIVERSAL SETTINGS -------------------------------------------------------
        # dropdown box for signal windows
        self.windowSelect = QtWidgets.QComboBox()
        self.windowSelect.addItems(['blackman', 'hamming',
                                    'hann', 'bartlett', 'flattop', 'parzen',
                                    'bohman', 'blackmanharris', 'nuttall',
                                    'barthann', 'boxcar', 'triang'])
        self.windowSelect.currentTextChanged.connect(self.save_settings)
        self.windowLabel = QtWidgets.QLabel('Window:')

        # Band pass slider
        self.bandPassSlider = QRangeSlider()
        self.bandPassSlider.setRange(self.lowcut * 1e-3, self.highcut * 1e-3)
        self.bandPassSlider.setOrientation(Qt.Horizontal)
        # self.bandPassSlider.setTickInterval(20)
        self.bandPassSlider.valueChanged.connect(self.save_settings)
        self.bandpass_min = self.lowcut  # keeps track of current minimum bandpass value
        self.bandpass_max = self.highcut  # # keeps track of current maximum bandpass value

        self.highcutLabel = QtWidgets.QLabel('{} kHz'.format(self.bandpass_min))
        self.lowcutLabel = QtWidgets.QLabel('{} kHz'.format(self.bandpass_max))
        self.bandPassLabel = QtWidgets.QLabel('Band-pass Range')

        # default settings button
        self.defaultBtn = QtWidgets.QPushButton('Default')
        self.defaultBtn.clicked.connect(self.defaultSettings)

        # return to information display
        self.closeBtn = QtWidgets.QPushButton('Close')
        self.closeBtn.clicked.connect(self.display_info)

        # SPECTROGRAM SETTINGS -------------------------------------------------------
        # widget for grouping items w/ grid layout
        self.sgSettingsWgt = QtWidgets.QWidget()
        self.mainLayout.addWidget(self.sgSettingsWgt, 1, 0, 1, 2)
        self.sgSettingsLayout = QtWidgets.QGridLayout()
        self.sgSettingsWgt.setLayout(self.sgSettingsLayout)

        # radio buttons for switching between logspace and linspace
        self.logspace_button = QtWidgets.QRadioButton('Logspace')
        self.logspace_button.toggled.connect(self.save_settings)
        self.logspace_button.toggled.connect(self.delete_cbar)
        self.logspace_button.toggled.connect(self.create_cbar)
        self.linspace_button = QtWidgets.QRadioButton('Linspace')
        self.linspace_button.toggled.connect(self.save_settings)
        self.linspace_button.toggled.connect(self.delete_cbar)
        self.linspace_button.toggled.connect(self.create_cbar)

        # dropdown box for length of window (powers of 2)
        self.lengthSelect = QtWidgets.QComboBox()
        self.lengthSelect.addItems(['128', '256', '512', '1024', '2048',
                                    '4096', '8192'])
        self.lengthSelect.setCurrentText('256')
        self.lengthSelect.currentTextChanged.connect(self.save_settings)
        self.lengthLabel = QtWidgets.QLabel("Length:")

        # slider for adjusting overlap (as % of window length)
        self.noverlapSlider = QtWidgets.QSlider(orientation=2)  # 2 = vertical
        self.noverlapSlider.setRange(20, 99)
        # self.noverlapSlider.setSingleStep(int(self.nperseg/10)) #unknown
        self.noverlapSlider.setPageStep(10)
        self.noverlapSlider.setSliderPosition(int(100*self.noverlap/self.nperseg))
        self.noverlapSlider.valueChanged.connect(self.save_settings)

        # slider for adjusting minAmp
        self.minAmpSlider = QtWidgets.QSlider(orientation=1)
        self.minAmpSlider.setRange(self.truncated_amp_val, 0)
        self.minAmpSlider.setSliderPosition(self.truncated_amp_val)
        self.minAmpSlider.valueChanged.connect(self.save_settings)

        minAmp_disp = self.minAmpSlider.sliderPosition() # number to display
        self.minAmpLabel = QtWidgets.QLabel('Truncation: {}dB'.format(str(minAmp_disp)))

        noverlap_disp = self.noverlapSlider.sliderPosition()
        self.noverlapLabel = QtWidgets.QLabel('overlap: {}%'.format(str(noverlap_disp)))

        # SPECTRUM SETTINGS -------------------------------------------------------
        # widget for grouping items w/ grid layout
        self.smSettingsWgt = QtWidgets.QWidget()
        self.mainLayout.addWidget(self.smSettingsWgt, 1, 0, 1, 2)
        self.smSettingsLayout = QtWidgets.QGridLayout()
        self.smSettingsWgt.setLayout(self.smSettingsLayout)

        # range slider
        self.dbSlider = QtWidgets.QSlider(orientation=1)  # 1 = horizontal
        self.dbSlider.setRange(0, 70)
        # self.dbSlider.setSingleStep(int(self.nperseg/10)) #unknown
        self.dbSlider.setPageStep(10)
        self.dbSlider.setSliderPosition(self.dBCutoff)
        self.dbSlider.valueChanged.connect(self.save_settings)
        self.dbLabel = QtWidgets.QLabel("Range: {} dB".format(self.dBCutoff))

        # OSCILLOGRAM SETTINGS -------------------------------------------------------
        # the main widget
        self.ogSettingsWgt = QtWidgets.QWidget()
        self.mainLayout.addWidget(self.ogSettingsWgt, 1, 0, 1, 2)
        self.ogSettingsLayout = QtWidgets.QGridLayout()
        self.ogSettingsWgt.setLayout(self.ogSettingsLayout)

    def update_labels(self):
        """
        Updates the labels in the information menu
        """

        self.nString = 'N: {}'.format(self.n)
        minutes = self.tPassed//60
        sec = self.tPassed % 60
        self.tString = 't: {m:.0f} min {sec:.2f} sec'.format(m=minutes,sec=sec)
        self.samplingFrequencyString = 'Fs: {:.0f} kHz'.format(self.samplingFrequency / 1000)
        self.iterationLabel.setText(self.nString)
        self.durationLabel.setText(self.tString)
        self.rateLabel.setText(self.samplingFrequencyString)
        self.fpsString = 'fps: {:.1f} Hz'.format(self.fps)
        self.fpsLabel.setText(self.fpsString)

    def save_settings(self):
        """
        Updates the labels on the settings menu and updates the plots accordingly
        """

        self.sigWindow = self.windowSelect.currentText()
        self.nperseg = int(self.lengthSelect.currentText())
        # print(self.nperseg)
        self.noverlap = (self.noverlapSlider.sliderPosition()/100)*self.nperseg
        # print(self.noverlap)
        self.nfft = self.nperseg  # can be changed
        noverlapDisp = self.noverlapSlider.sliderPosition()
        self.noverlapLabel.setText('overlap: {}%'.format(str(noverlapDisp)))

        self.bandpass_min = self.bandPassSlider.value()[0]
        self.bandpass_max = self.bandPassSlider.value()[1]

        self.lowcutLabel.setText('{} kHz'.format(self.bandpass_min))
        self.highcutLabel.setText('{} kHz'.format(self.bandpass_max))

        self.dBCutoff = self.dbSlider.sliderPosition()
        self.dbLabel.setText("Range: {} dB".format(self.dBCutoff))

        minAmp_disp = self.minAmpSlider.sliderPosition()  # number to display
        self.minAmpLabel.setText('Truncation: {}dB'.format(str(minAmp_disp)))

        self.select_space()

        self.reload_plots()

    def defaultSettings(self):
        """
        The default settings the program starts in
        """

        self.sigWindow = 'blackman'
        self.windowSelect.setCurrentText(self.sigWindow)
        self.nperseg = 256
        self.lengthSelect.setCurrentText(str(self.nperseg))
        self.noverlap = self.nperseg//5
        self.noverlapSlider.setSliderPosition(100*self.noverlap/self.nperseg)
        self.nfft = self.nperseg

        self.lowcut = 20e3
        self.highcut = 100e3
        self.bandpass_min = self.lowcut * 1e-3
        self.bandpass_max = self.highcut * 1e-3
        self.bandPassSlider.setValue((self.bandpass_min, self.bandpass_max))
        self.lowcutLabel.setText('{} kHz'.format(self.bandpass_min))
        self.highcutLabel.setText('{} kHz'.format(self.bandpass_max))

        self.minAmpSlider.setSliderPosition(self.truncated_amp_val)

        self.reload_plots()

    def reload_plots(self):
        """
        Updates the plots based on current settings
        """

        lamp, ramp = self.amplitude(self.file)    # Find data source and switch to web server file request
        if self.sgBtn.isChecked():
            self.build_sg(self.leftPlot, self.rightPlot, lamp, ramp)
        elif self.smBtn.isChecked():
            self.build_sm(self.leftPlot, self.rightPlot, lamp, ramp)
        elif self.ogBtn.isChecked():
            self.build_og(self.leftPlot, self.rightPlot, lamp, ramp)

    def display_info(self):
        """
        Displays the main information screen
        """

        self.sgSettingsWgt.hide()
        self.smSettingsWgt.hide()
        self.ogSettingsWgt.hide()
        self.infoWidget.show()

    def disp_spectrogram_settings(self):
        """
        Displays the spectrogram settings menu
        """

        center = Align.CENTER.value
        right = Align.RIGHT.value

        self.sgSettingsLayout.addWidget(self.logspace_button, 1, 0)
        self.sgSettingsLayout.addWidget(self.linspace_button, 2, 0)

        self.sgSettingsLayout.addWidget(self.windowLabel, 3, 0)
        self.sgSettingsLayout.addWidget(self.windowSelect, 3, 1)

        self.sgSettingsLayout.addWidget(self.lengthLabel, 4, 0)
        self.sgSettingsLayout.addWidget(self.lengthSelect, 4, 1)

        self.sgSettingsLayout.addWidget(self.noverlapLabel, 0, 2, center)
        self.sgSettingsLayout.addWidget(self.noverlapSlider, 1, 2, 3, 1, center)
        # self.sgSettingsLayout.setRowMinimumHeight(1, 50)

        self.sgSettingsLayout.addWidget(self.lowcutLabel, 0, 3)
        self.sgSettingsLayout.addWidget(self.highcutLabel, 0, 5, right)
        self.sgSettingsLayout.addWidget(self.bandPassLabel, 0, 4, center)
        self.sgSettingsLayout.addWidget(self.bandPassSlider, 1, 3, 1, 3)

        self.sgSettingsLayout.addWidget(self.minAmpLabel, 2, 3)
        self.sgSettingsLayout.addWidget(self.minAmpSlider, 3, 3, 1, 3)

        self.sgSettingsLayout.setColumnMinimumWidth(4, 20)
        self.sgSettingsLayout.setColumnMinimumWidth(6, 20)
        self.sgSettingsLayout.addWidget(self.defaultBtn, 0, 7)
        self.sgSettingsLayout.addWidget(self.closeBtn, 1, 7)

        self.infoWidget.hide()
        self.sgSettingsWgt.show()

    def disp_spectrum_settings(self):
        """
        Displays the spectrum settings menu
        """

        right = Align.RIGHT.value
        center = Align.CENTER.value

        self.smSettingsLayout.addWidget(self.windowLabel, 0, 0)
        self.smSettingsLayout.addWidget(self.windowSelect, 0, 1)
        self.smSettingsLayout.addWidget(self.defaultBtn, 0, 6)
        self.smSettingsLayout.addWidget(self.closeBtn, 1, 6)
        self.smSettingsLayout.addWidget(self.dbLabel, 0, 2)
        self.smSettingsLayout.addWidget(self.dbSlider, 1, 2, 1, 3)
        self.smSettingsLayout.addWidget(self.lowcutLabel, 2, 1, right)
        self.smSettingsLayout.addWidget(self.bandPassLabel, 2, 0, 1, 2, center)
        self.smSettingsLayout.addWidget(self.highcutLabel, 2, 6)
        self.smSettingsLayout.addWidget(self.bandPassSlider, 2, 2, 1, 3)

        self.infoWidget.hide()
        self.smSettingsWgt.show()

    def disp_oscillogram_settings(self):
        """
        Displays the oscillogram settings menu
        """

        right = Align.RIGHT.value

        self.ogSettingsLayout.addWidget(self.defaultBtn, 0, 6)
        self.ogSettingsLayout.addWidget(self.closeBtn, 1, 6)

        self.infoWidget.hide()
        self.ogSettingsWgt.show()

    # START OF DATA PROCESSING

    def amplitude(self, filename):
        """
        Calculates the amplitudes of the left and right plots
        :param filename: the data file that will be accessed
        :return: the left array and right array with the respective amplitudes
        """

        data = []
        l_data = []
        r_data = []
        file = open(filename, newline='\n')    # http request to retrieve file

        # TODO: figure out why i - 0, what 1.65 - (-1.65) means, why divide by 4095, MAKE INTO VARIABLES

        # iterating through all ints in the file
        for data_point in file:
            i = int(data_point.strip())
            j = (i - 0) * (1.65 - (-1.65)) / 4095
            data.append(j)

        # getting averages of left and right plots
        mid_index = len(data) // 2
        l_avg = sum(data[:self.numDataPoints]) / mid_index
        r_avg = sum(data[self.numDataPoints:]) / mid_index

        # calculate amplitudes of left and right plots
        for data_point in range(mid_index):
            l_data.append(data[data_point] - l_avg)
            if self.truncated_amp_val > data_point:  # attempting to get lowest amplitude value
                self.truncated_amp_val = data_point
        for data_point in range(mid_index, len(data)):
            r_data.append(data[data_point] - r_avg)
            if self.truncated_amp_val > data_point:
                self.truncated_amp_val = data_point

        return l_data, r_data

    def butter_bandpass(self, lowcut, highcut, fs, order=5):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')

        return b, a

    def butter_bandpass_filter(self, data, lowcut, highcut, fs, order=5):
        """
        Used to build the oscillogram in build_og().
        """

        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def delete_cbar(self):
        """
        Removes the left and right colorbars from the plots, and changes the colorbar objects to noneType.
        """
        # Needs to have no parameters so it can be used by linspace / logspace button.connect
        if self.lCbar != None:
            self.leftPlot.layout.removeItem(self.lCbar)
            self.lCbar = None  # solves problem with "ghost colorbar" in top left corner of plot
        if self.rCbar != None:
            self.rightPlot.layout.removeItem(self.rCbar)
            self.rCbar = None  # solves problem with "ghost colorbar" in top left corner of plot
    
    def create_cbar(self):
        """
        Creates the left and right colorbars and inserts them in the plots.
        """
        # Needs to have no parameters so it can be used by linspace / logspace button.connect
        if self.lCbar == None:
            if self.logspace_selected:
                self.lCbar = ColorBarItem( values= (self.minAmp, 0), cmap = pg.colormap.get('CET-L9'), interactive= False, label= "Amplitude (dB)", width= 15)
                self.lCbar.setImageItem(self.lImg, insert_in= self.leftPlot)
            else:
                self.lCbar = ColorBarItem( values= (0, 1), cmap = self.cmap, interactive= False, label= "Amplitude", width= 15)
                self.lCbar.setImageItem(self.lImg, insert_in= self.leftPlot)
        if self.rCbar == None:
            if self.logspace_selected:
                self.rCbar = ColorBarItem( values= (self.minAmp, 0), cmap = pg.colormap.get('CET-L9'), interactive= False, label= "Amplitude (dB)", width = 15)
                self.rCbar.setImageItem(self.rImg, insert_in= self.rightPlot)
            else:
                self.rCbar = ColorBarItem( values= (0, 1), cmap = self.cmap, interactive= False, label= "Amplitude", width = 15)
                self.rCbar.setImageItem(self.rImg, insert_in= self.rightPlot)
    
    def linscale_cbar(self):
        """
        Changes the colorbar values to a linear scale.
        """

        if self.lCbar != None:
            self.lCbar.setLevels(values= (0,1) )
        if self.rCbar != None:
            self.rCbar.setLevels(values= (0,1) )

    def build_og(self, leftPlot, rightPlot, lamp, ramp):
        """
        Builds the oscillogram plot
        :param leftPlot: the PlotItem for the left plot
        :param rightPlot: the PlotItem for the right plot
        :param lamp: the array of amplitudes for the left plot
        :param ramp: the array of amplitudes for the right plot
        """

        leftPlot.clear()
        rightPlot.clear()
        self.delete_cbar()

        feedback_cutoff = 15  # TODO: add comment definition of this variable
        time_array = self.timeArray[feedback_cutoff:]

        lamp = self.butter_bandpass_filter(lamp, self.lowcut,
                                           self.highcut, self.samplingFrequency)
        ramp = self.butter_bandpass_filter(ramp, self.lowcut,
                                           self.highcut, self.samplingFrequency)

        lamp = lamp[feedback_cutoff:]
        ramp = ramp[feedback_cutoff:]
        ramp = ramp * 1000 # scales right amplitude to be same units as left plot
        leftPlot.plot(x=time_array, y=lamp)
        rightPlot.plot(x=time_array, y=ramp)
        leftPlot.setLabel('bottom', 'Time', units='s')
        rightPlot.setLabel('bottom', 'Time', units='s')

        # attempts to fix SI Prefix issues
        # laxis = leftPlot.getAxis('left')
        # laxis.enableAutoSIPrefix(False)
        # leftPlot.setAxisItems({'left': laxis})

        leftPlot.setLabel('left', 'Amplitude', units='V')

        leftPlot.autoRange()
        rightPlot.autoRange()

    def fourier_transform(self, lamp, ramp):
        """
        Used to build the spectrum in build_sm(). Fourier transforms the parameter amplitude arrays
        :param lamp: amplitude array for the left plot
        :param ramp: amplitude array for the right plot
        :return: the left array and right array of the decibel values for the respective plots
        """

        window = self.window_selection()
        lfreq = np.abs(np.fft.rfft(lamp * window))
        rfreq = np.abs(np.fft.rfft(ramp * window))

        lowcut = int((self.lowcut/self.maxFreq) * len(lfreq))  # first data point is 0 and offsets rest of data
        highcut = int((self.highcut/self.maxFreq) * len(lfreq))
        lyf = lfreq[lowcut:highcut]  # TODO: figure out what the 'y' in lyf/ryf means
        ryf = rfreq[lowcut:highcut]
        yf = np.concatenate((lyf, ryf))

        base_dB = max(yf)
        ldB = []
        rdB = []

        for data_point in lyf:
            ldB.append(20 * np.log10(data_point/base_dB))  # convert to dB scale
        for data_point in ryf:
            rdB.append(20 * np.log10(data_point/base_dB))  # convert to dB scale

        return ldB, rdB

    def build_sm(self, leftPlot, rightPlot, lamp, ramp):
        """
        Builds the spectrum plot
        :param leftPlot: the PlotItem for the left plot
        :param rightPlot: the PlotItem for the right plot
        :param lamp: the array of amplitudes for the left plot
        :param ramp: the array of amplitudes for the right plot
        """

        leftPlot.clear()
        rightPlot.clear()
        self.delete_cbar()
        
        l_dB, r_dB = self.fourier_transform(lamp, ramp)

        x_axis_frequency = np.linspace(0.0, self.samplingFrequency // 2, self.numDataPoints // 2)
        lowcut = int((self.lowcut / self.maxFreq) * len(x_axis_frequency))
        highcut = int((self.highcut / self.maxFreq) * len(x_axis_frequency))
        x_axis_frequency = x_axis_frequency[lowcut:highcut]
        # x_axis_frequency = x_axis_frequency[self.N//20:self.N//4]
        # yf = freq[self.N//20:self.N//4] #from 20kHz to 100kHz
        leftPlot.plot(x=x_axis_frequency, y=l_dB)
        rightPlot.plot(x=x_axis_frequency, y=r_dB)

        leftPlot.setLabel('bottom', 'Frequency', units='Hz')
        rightPlot.setLabel('bottom', 'Frequency', units='Hz')
        leftPlot.setLabel('left', 'Amplitude', units='dB')

        leftPlot.autoRange()
        rightPlot.autoRange()
        leftPlot.setRange(xRange=(self.bandpass_min * 1e3, self.bandpass_max * 1e3), yRange=(-1*self.dBCutoff, 0))
        rightPlot.setRange(xRange=(self.bandpass_min * 1e3, self.bandpass_max * 1e3), yRange=(-1*self.dBCutoff, 0))

        # rows = len(lyf)
        # columns = len(x_axis_frequency)
        # yscale = 1
        # xscale = 200/columns
        # xaxis = pg.AxisItem('bottom', text='Frequency', units='Hz')
        # xaxis.setRange(0, 200e3)
        # leftPlot.setAxisItems({'bottom': xaxis})
        # leftPlot.scale(xscale, yscale)
        # rightPlot.scale(xscale, yscale)

    def build_sg(self, left_plot, right_plot, ldata, rdata):
        """
        Builds the spectrogram plot
        :param left_plot: the PlotItem for the left plot
        :param right_plot: the PlotItem for the right plot
        :param ldata: the array of amplitudes for the left plot
        :param rdata: the array of amplitudes for the right plot
        """

        left_plot.clear()
        right_plot.clear()

        ldata = np.array(ldata)
        rdata = np.array(rdata)

        # most efficient nperseg & nfft values are powers of 2
        # spectrogram returns to Sxx as (frequency x time)
        f, t, lSxx = spectrogram(ldata, self.samplingFrequency, self.sigWindow,
                                 nperseg=self.nperseg,
                                 noverlap=self.noverlap,
                                 nfft=self.nfft)
        f, t, rSxx = spectrogram(rdata, self.samplingFrequency, self.sigWindow,
                                 nperseg=self.nperseg,
                                 noverlap=self.noverlap,
                                 nfft=self.nfft)

        Sxx = np.concatenate([lSxx,rSxx])
        
        # if self.logspace_selected:
        # print(lSxx)
        # print(np.min(Sxx))
        
        if self.logspace_selected:
            lSxx = 20 * np.log10(lSxx/np.max(Sxx))  # left amplitude values
            rSxx = 20 * np.log10(rSxx/np.max(Sxx))  # right amplitude values
        else:
            lSxx = lSxx / np.max(lSxx)
            rSxx = rSxx / np.max(rSxx)
            
        # print(lSxx)
        

        self.lImg = pg.ImageItem()  # spectrogram is an image
        self.rImg = pg.ImageItem()
        left_plot.addItem(self.lImg)  # add sg to widget
        right_plot.addItem(self.rImg)

        # color mapping     
        nColors = 63


        color = cm.get_cmap('viridis', 63)
        #color arr
        colorMapArr=[]
        for i in range(0, 63):
            colorMapArr.append(color(i))
        print(len(colorMapArr[0]))
        # pos = [x**(1/3) for x in pos]
        if self.logspace_selected:
            pos = np.logspace(-0.5, 0, nColors)
        else:
            pos = np.linspace(0, 1, nColors)
        #color = cm.ice.discrete(63)
        colorArr = {}
        colorFunc = cm.get_cmap('hsv', 256)
        for i in range(63):
            colorArr[i]=(colorFunc(i))
        
        self.cmap = pg.ColorMap(pos, colorArr)
        

        self.lut = self.cmap.getLookupTable(0.0, 1.0, 256)
        self.lImg.setLookupTable(self.lut)
        self.rImg.setLookupTable(self.lut)

        # cutting the frequency range to 20kHz-100kHz
        fRes = (f[1] - f[0])/2
        fLowcut = self.binary_search(f, self.lowcut, fRes)
        fHighcut = self.binary_search(f, self.highcut, fRes)
        lSxx = lSxx[fLowcut:fHighcut]
        rSxx = rSxx[fLowcut:fHighcut]
        f = f[fLowcut:fHighcut]

        # Truncating the low amplitudes
        i = 0
        j = 0
        self.minAmp = self.minAmpSlider.sliderPosition()  # minimum amplitude value shown
        while i < len(lSxx):
            while j < len(lSxx[i]):
                if lSxx[i][j] < self.minAmp:
                    lSxx[i][j] = self.truncated_amp_val
                if rSxx[i][j] < self.minAmp:
                    rSxx[i][j] = self.truncated_amp_val
                j += 1
            i += 1
            j = 0

        # empty array for the image
        rows = len(f)
        columns = len(t)
        self.lImg_array = np.zeros((rows, columns))
        self.rImg_array = np.zeros((rows, columns))

        # scale the axes
        yscale = (self.highcut - self.lowcut )*1e-3 / rows  # scale is wrong
        xscale = max(t)/columns

        # Changes image orientation and shifts image to be in frame
        tr = QtGui.QTransform()
        tr.translate(0, 20)  # shifts graph up to make y-axis values 20-100 kHz
        tr.scale(xscale, yscale)  # makes image fit plot
        tr.scale(-1, 1)  # mirrors image
        tr.rotate(90)  # corrects image orientation
        self.lImg.setTransform(tr)
        self.rImg.setTransform(tr)

        # normalize together
        nSxx = np.concatenate([lSxx, rSxx])
        min_level = nSxx.min()
        max_level = nSxx.max()
        self.lImg.setImage(lSxx, autoLevels=True, levels=(min_level, max_level))  # level values are estimated from appearance
        self.rImg.setImage(rSxx, autoLevels=True, levels=(min_level, max_level))

        left_plot.autoRange()
        right_plot.autoRange()
        left_plot.setRange(yRange=(self.bandpass_min, self.bandpass_max))
        right_plot.setRange(yRange=(self.bandpass_min, self.bandpass_max))

        left_plot.setLabel('bottom', 'Time', units='s')
        right_plot.setLabel('bottom', 'Time', units='s')
        left_plot.setLabel('left', 'Frequency', units='kHz')

        self.create_cbar()  # creates the colorbar if the object isn't noneType

        # Changes the amplitude values according to truncation slider
        if self.logspace_selected:
            self.lCbar.setLevels(low= self.minAmp)
            self.rCbar.setLevels(low= self.minAmp)

    def run(self):
        # startTime = time.time()
        self.startBtn.setStyleSheet("background-color: red")
        self.deformBtn.setStyleSheet("background-color: red")
        jStr = self.iterationInput.text()
        try:
            self.j = int(jStr)
        except ValueError:
            self.j = 'inf'
        basepath = '/home/devan/Coding/MuellerLab/BatBotGUI/Data'
        # /home/devan/BatBot/Data # for Jetson
        data = []
        with os.scandir(basepath) as entries:
            for entry in entries:
                if entry.is_file():
                    data.append(entry.path)
        while self.startBtn.isChecked():
            if self.j != 'inf':
                # self.n initialized in __init__
                if self.n >= self.j:
                    break
            tStart = time.time()
            if self.useUpload == False:
                self.file = data[self.n]
            # tAmp = time.time()
            lamp, ramp = self.amplitude(self.file)  # if useUpload True, file is set in waveformUpload()
            # print("Amplitude: {}".format(time.time() - tAmp))
            if self.sgBtn.isChecked():
                # tSg = time.time()
                self.build_sg(self.leftPlot, self.rightPlot, lamp, ramp)
                # print("Spectrogram: {}".format(time.time() - tSg))
            elif self.smBtn.isChecked():
                # tSm = time.time()
                self.build_sm(self.leftPlot, self.rightPlot, lamp, ramp)
                # print("Spectrum: {}".format(time.time() - tSm))
            elif self.ogBtn.isChecked():
                # tOg = time.time()
                self.build_og(self.leftPlot, self.rightPlot, lamp, ramp)
                # self.build_og(self.rightPlot, self.timeArray, ramp, 'right')
                # print("Oscillogram: {}".format(time.time() - tOg))
            # self.tPassed = time.time() - startTime
            tOnce = time.time() - tStart
            self.fps = 1/tOnce
            self.update_labels()
            # self.fpsUpdate() #temporary
            self.n += 1
            
            app.processEvents()
        self.n = 0
        self.useUpload = False
        # endTime = time.time()
        # self.tTotal = endTime - startTime
        # print("Total: {}".format(self.tTotal))
        # startTime = endTime
        self.startBtn.setStyleSheet("background-color: green")
        self.startBtn.setChecked(False)
        

    def select_space(self):
        """
        Keeps track of whether the logspace button is selected or not, updates the spectrogram plots
        """

        if self.logspace_button.isChecked():
            self.logspace_selected = True
        else:
            self.logspace_selected = False

    def plotSelection(self):
        self.whichSettings()
        lamp, ramp = self.amplitude(self.file)
        if self.sgBtn.isChecked():
            self.build_sg(self.leftPlot, self.rightPlot, lamp, ramp)
        elif self.smBtn.isChecked():
            self.build_sm(self.leftPlot, self.rightPlot, lamp, ramp)
        elif self.ogBtn.isChecked():
            self.build_og(self.leftPlot, self.rightPlot, lamp, ramp)

    def whichSettings(self):
        self.menuBtn.disconnect()
        if self.sgBtn.isChecked():
            self.menuBtn.clicked.connect(self.disp_spectrogram_settings)
        elif self.smBtn.isChecked():
            self.menuBtn.clicked.connect(self.disp_spectrum_settings)
        elif self.ogBtn.isChecked():
            self.menuBtn.clicked.connect(self.disp_oscillogram_settings)

    def waveformUpload(self):
        filepath = QtWidgets.QFileDialog.getOpenFileName(self, 'Open file',
                                                     '/home/devan/BatBot/Data',
                                                     "Text Files (*.txt)")  # Jetson filepath
        self.file = filepath[0]
        self.useUpload = True
        self.reload_plots()

    def binary_search(self, data, target, res):
        first = 0
        last = len(data) - 1
        index = -1
        while (first <= last) and (index == -1):
            mid = (first+last)//2
            if data[mid] >= target - res and data[mid] <= target + res:
                index = mid
            else:
                if target < data[mid]:
                    last = mid - 1
                else:
                    first = mid + 1
        return index

    def window_selection(self):
        """
        Returns the respective window based on the current signal window
        """

        if self.sigWindow == 'blackman':
            return np.blackman(self.numDataPoints)
        elif self.sigWindow == 'hamming':
            return np.hamming(self.numDataPoints)
        elif self.sigWindow == 'hann':
            return np.hanning(self.numDataPoints)
        elif self.sigWindow == 'bartlett':
            return np.bartlett(self.numDataPoints)
        elif self.sigWindow == 'flattop':
            return signal.flattop(self.numDataPoints)
        elif self.sigWindow == 'parzen':
            return signal.parzen(self.numDataPoints)
        elif self.sigWindow == 'bohman':
            return signal.bohman(self.numDataPoints)
        elif self.sigWindow == 'blackmanharris':
            return signal.blackmanharris(self.numDataPoints)
        elif self.sigWindow == 'nuttall':
            return signal.nuttall(self.numDataPoints)
        elif self.sigWindow == 'barthann':
            return signal.barthann(self.numDataPoints)
        elif self.sigWindow == 'boxcar':
            return signal.boxcar(self.numDataPoints)
        elif self.sigWindow == 'triang':
            return signal.triang(self.numDataPoints)
        else:
            print("Invalid window selected!")
            quit()


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    w = Window()
    w.show()
    # w.showMaximized() # for Jetson
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()