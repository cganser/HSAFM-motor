#!/home/afm/python/daq_venv/bin/python

#Software to control the approach/retract stepper motor for HS-AFM systems.
#Built for Raspberry Pi 5 equipped with the following: MCC 118 DAQ-HAT & MCC 152 DAQ-HAT
#
#In order to enable hardware PWM to directly send pulses to the stepper motor, make sure that the following line
#is added to /boot/firmware/config.txt: 
#
#       dtoverlay=pwm-2chan
#
#The two hardware PWM channels are GPIO18 (pin nr. 12) and GPIO19 (pin nr.35) on the Raspberry Pi 5 GPIO.
#(see /boot/firmware/overlays/README for details)
#Christian Ganser, 2025


import sys
import os
import time
import struct
import matplotlib
import array
import gc
import math
import numpy as np
matplotlib.use('Qt5Agg')

from daqhats import mcc118, mcc152, OptionFlags, HatIDs, HatError, hat_list, DIOConfigItem

from rpi_hardware_pwm import HardwarePWM

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import QSize, Qt, QObject, QThread, pyqtSignal, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class hat_device():
    def __init__(self,hType):
        self.options = OptionFlags.DEFAULT
        self.type = hType
        if self.type == "mcc118":
            self.id = HatIDs.MCC_118
        elif self.type == "mcc152":
            self.id = HatIDs.MCC_152
        else:
            self.id == None

        if self.id == None:
            raise ValueError("ERROR: Invalid HAT type. Please specify either \"mcc118\" or \"mcc152\"!")

        self.address = None
        self.hat = None

    def select_hat(self,n):
        hats = hat_list(filter_by_id=self.id)
        nHats = len(hats)


        if nHats < 1:
            raise HatError(0, "ERROR: No HAT devices found!")
        elif nHats == 1:
            self.address = hats[0].address
        else:
            if n <= nHats:
                self.address = hats[n].address
            else:
                raise ValueError("ERROR: Invalid HAT selection!")

        if self.address == None:
            raise ValueError("ERROR: No HAT could be selected!")

        if self.type == "mcc118":
            self.hat = mcc118(self.address)
        elif self.type == "mcc152":
            self.hat = mcc152(self.address)
        else:
            self.hat = None




class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        #gc.enable()

        #select and initialize the AD HAT
        self.ADHat = hat_device("mcc118")
        self.ADHat.select_hat(0)

        self.sumPatch = None
        self.defPatch = None
        self.ampPatch = None
        self.zpiPatch = None

        #select and intitalze the DA HAT
        self.DAHat = hat_device("mcc152")
        self.DAHat.select_hat(0)
        self.DAHat.hat.dio_reset()
        #self.DAHat.hat.dio_config_write_bit(0,DIOConfigItem.DIRECTION,0)
        self.DAHat.hat.dio_output_write_bit(0,0)

        self.forceOffset = 2.5

        self.DAHat.hat.a_out_write(0,self.forceOffset)
        self.DAHat.hat.a_out_write(1,0.0)

        self.ADUpdateTimeMS = 2 #how many milliseconds between data acquisition
        self.graphUpdateTimeMS = 50 #how many millisecond between updating the bar graphs
        self.currGraphCount = 0

        self.fastMoveFreq = 15000
        self.slowMoveFreq = 3000
        self.autoApproachFreq = 5000
        self.acceleration = 1000
        self.dSpeed = 1000
        self.powerCycle = 50

        self.maxTravelFast = 500000
        self.maxTravelSlow = 10000

        self.fastLimitState = 0
        self.slowLimitState = 0

        self.accelFreq = 0

        self.directionChn = 0
        self.singleChn = 2

        self.approachChn = 2
        self.retractChn = 3

        self.sumV = 0
        self.defV = 0
        self.ampV = 0
        self.zpiV = 0

        self.sumChn = 0
        self.defChn = 1
        self.ampChn = 2
        self.zpiChn = 3

        self.initialAmp = 0
        self.initialDef = 0
        self.defLimit = 1.0
        self.zpiLimit = 1.0
        self.ampRatio = 0.5

        self.outputMode = 0     #0: 2 pwm channels; 
                                #1: 1 pwm channel, 1 direction DIO channel

        self.meterRunning = True

        self.motorPos = 0
        self.motorUpdateTimeMS = 10
        self.pulseFreq = 0
        self.motorDirection = 1 #approach = -1, rectract = 1
        self.motorRunning = False
        self.autoApproach = False

        self.startPos = 0
        self.motorDist = 0
        self.slowTravel = 0
        self.fastTravel = 0

        self.LoadSettings()
        #After an OS update (April 2025), the value for "chip" has to be 0, otherwise it will not work.
        #The manual on the homepage for the rpi_hardware_pwm package originally stated that for RPi5, "chip" should be 2.
        self.chip = 0
        self.pwm_ccw = HardwarePWM(pwm_channel=self.approachChn, hz=self.slowMoveFreq, chip=self.chip)
        self.pwm_cw = HardwarePWM(pwm_channel=self.retractChn, hz=self.slowMoveFreq, chip=self.chip)

        self.pwm_ccw.stop()
        self.pwm_cw.stop()

        self.createCentralWidget()
        self.setCentralWidget(self.centralFrame)
        self.setWindowTitle("Motor Control")
        self.createMenuBar()


    def createCentralWidget(self):
        self.centralFrame = QtWidgets.QFrame()
        self.tabs = QtWidgets.QTabWidget()
        self.controlTab = QtWidgets.QWidget()
        self.settingTab = QtWidgets.QWidget()
        self.advancedTab = QtWidgets.QWidget()
        self.forceTab = QtWidgets.QWidget()
        self.tabs.addTab(self.controlTab, "Control")
        self.tabs.addTab(self.settingTab, "Settings")
        self.tabs.addTab(self.advancedTab, "Advanced")
        self.tabs.addTab(self.forceTab, "Force Curve")

        self.layout = QtWidgets.QVBoxLayout(self.centralFrame)

        self.layout.addWidget(self.tabs)

        self.ControlTab()
        self.SettingsTab()
        self.AdvancedTab()
        self.ForceTab()

    def ControlTab(self):
        layout = QtWidgets.QGridLayout(self.controlTab)
        meterLayout = QtWidgets.QGridLayout()
        motorLayout = QtWidgets.QGridLayout()
        self.bg_color = self.centralFrame.palette().color(QtGui.QPalette.Window).name()

        #Meter (sum,deflection,z-position)
        #
        self.MeterBox = QtWidgets.QGroupBox()

        self.MeterStopButton= QtWidgets.QPushButton("Stop\n Meter", clicked=self.MeterStopButtonFunction)
        self.sumLabel = QtWidgets.QLabel("Sum:")
        self.deflectionLabel = QtWidgets.QLabel("Deflection:")
        self.amplitudeLabel = QtWidgets.QLabel("Amplitude:")
        self.zPiezoLabel = QtWidgets.QLabel("z-Piezo:")

        self.sumValue = QtWidgets.QDoubleSpinBox()
        self.sumValue.setReadOnly(1)
        self.sumValue.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.sumValue.setRange(-10.0,10.0)
        self.deflectionValue = QtWidgets.QDoubleSpinBox()
        self.deflectionValue.setReadOnly(1)
        self.deflectionValue.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.deflectionValue.setRange(-10.0,10.0)
        self.amplitudeValue = QtWidgets.QDoubleSpinBox()
        self.amplitudeValue.setReadOnly(1)
        self.amplitudeValue.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.amplitudeValue.setRange(-10.0,10.0)
        self.zPiezoValue = QtWidgets.QDoubleSpinBox()
        self.zPiezoValue.setReadOnly(1)
        self.zPiezoValue.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.zPiezoValue.setRange(-10.0,10.0)

        self.canvas = FigureCanvas(Figure(figsize=(5,1.8)))

        layout.addWidget(self.sumLabel,0,0)
        layout.addWidget(self.sumValue,0,1)
        layout.addWidget(self.deflectionLabel,1,0)
        layout.addWidget(self.deflectionValue,1,1)
        layout.addWidget(self.amplitudeLabel,2,0)
        layout.addWidget(self.amplitudeValue,2,1)
        layout.addWidget(self.zPiezoLabel,3,0)
        layout.addWidget(self.zPiezoValue,3,1)

        layout.addWidget(self.MeterStopButton,0,6,4,1)

        layout.addWidget(self.canvas,0,2,4,4)

        self.MeterStopButton.setMinimumHeight(170)

        self.canvas.setMinimumHeight(180)
        self.canvas.setMinimumWidth(500)
        self.canvas.setMaximumWidth(500)

        self.MeterBox.setLayout(meterLayout)

        self.axes = self.canvas.figure.subplots()
        self.canvas.figure.set_layout_engine('constrained')
        self.canvas.figure.set_facecolor(self.bg_color)
        self.axes.set_xlim((-10.1,10.1))
        self.axes.set_ylim((-0.55,3.55))
        self.axes.axis('off')
        #This is the background of the plot
        bottom = -1
        left = -10.1
        width = 20
        height = 4.5
        self.BGPatch = self.axes.add_patch(matplotlib.patches.Rectangle((left,bottom),width,height,color="#DDDDDD"))
        self.BGPatch.set_animated(False)

        #Center Line
        self.axes.plot([0,0],[-0.5,0.5],color='#000000',ls=':',animated = False)
        self.axes.plot([0,0],[1.5,2.5],color='#000000',ls=':',animated = False)
        #Sparators
        self.axes.plot([-11,11],[0.5,0.5],color='#333333', animated = False)
        self.axes.plot([-11,11],[1.49,1.49],color='#333333', animated = False)
        self.axes.plot([-11,11],[2.49,2.49],color='#333333', animated = False)
        #Box
        self.axes.plot([-10.1,10.05,10.05,-10.1,-10.1],[-0.5,-0.5,3.5,3.5,-0.5],color='#333333',linewidth='2',animated = False)

        self.canvas.draw()

        self.FigBG = self.canvas.copy_from_bbox(self.canvas.figure.bbox)

        #
        #Meter (sum,deflection,z-position)
        self.sumColor = "#880099"
        self.defColor1 = "#004488"
        self.defColor2 = "#4488AA"
        self.ampColor = "#33AAAA"
        self.zpiColor1 = "#880000"
        self.zpiColor2 = "#AA4444"
        self.sumRect = matplotlib.patches.Rectangle((-10,2.65),10*0,0.7,color=self.sumColor)
        self.defRect = matplotlib.patches.Rectangle((0,1.65),0,0.7,color=self.defColor1)
        self.ampRect = matplotlib.patches.Rectangle((-10,0.65),10*0,0.7,color=self.ampColor)
        self.zpiRect = matplotlib.patches.Rectangle((0,-0.35),0,0.7,color=self.zpiColor1)

        self.sumPatch = self.axes.add_patch(self.sumRect)
        self.defPatch = self.axes.add_patch(self.defRect)
        self.ampPatch = self.axes.add_patch(self.ampRect)
        self.zpiPatch = self.axes.add_patch(self.zpiRect)

        self.sumPatch.set_animated(True)
        self.defPatch.set_animated(True)
        self.ampPatch.set_animated(True)
        self.zpiPatch.set_animated(True)

        self.axes.draw_artist(self.ampPatch)
        self.canvas.blit(self.canvas.figure.bbox)

        #Meter update timer
        self.ReadADTimer = QTimer()
        self.ReadADTimer.timeout.connect(self.updateADTimer)
        self.ReadADTimer.start(self.ADUpdateTimeMS)

        #Motor Control
        #

        self.AccelTimer = QTimer()
        self.AccelTimer.timeout.connect(self.AcceleratedMovement)
        self.AccelTimeMS = 10

        self.MotorBox = QtWidgets.QGroupBox("Motor")

        self.FastApproachButton = QtWidgets.QPushButton("Fast Approach", clicked=self.FastApproachButtonFunction)
        self.SlowApproachButton = QtWidgets.QPushButton("Slow Approach", clicked=self.SlowApproachButtonFunction)
        self.FastRetractButton = QtWidgets.QPushButton("Fast Retract", clicked=self.FastRetractButtonFunction)
        self.SlowRetractButton = QtWidgets.QPushButton("Slow Retract", clicked=self.SlowRetractButtonFunction)
        self.MotorStopButton = QtWidgets.QPushButton("Stop", clicked=self.MotorStopButtonFunction)

        self.SlowLimitToggle = QtWidgets.QCheckBox("Limit")

        if self.slowLimitState != 0:
            self.SlowLimitToggle.setChecked(True)
        else:
            self.SlowLimitToggle.setChecked(False)

        self.SlowLimitToggle.stateChanged.connect(self.SlowLimitCheckFunction)
        self.FastLimitToggle = QtWidgets.QCheckBox("Limit")

        if self.fastLimitState != 0:
            self.FastLimitToggle.setChecked(True)
        else:
            self.FastLimitToggle.setChecked(False)

        self.FastLimitToggle.stateChanged.connect(self.FastLimitCheckFunction)


        self.AutoApproachButton = QtWidgets.QPushButton("Auto\n Approach", clicked=self.AutoApproachButtonFunction)

        self.MotorPosLabel = QtWidgets.QLabel("Position:")
        self.MotorPosValue = QtWidgets.QLineEdit()
        self.MotorPosValue.setReadOnly(1)
        self.MotorPosValue.setText("0")


        self.MotorCurrSpeedLabel = QtWidgets.QLabel("Curr. Speed:")
        self.MotorCurrSpeedValue = QtWidgets.QLineEdit()
        self.MotorCurrSpeedValue.setText("0")
        self.MotorCurrSpeedValue.setReadOnly(1)

        self.MotorFasterButton = QtWidgets.QPushButton("Faster!", clicked=self.FasterButtonFunction)
        self.MotorSlowerButton = QtWidgets.QPushButton("Slower!", clicked=self.SlowerButtonFunction)


        motorLayout.addWidget(self.MotorPosLabel,0,0)
        motorLayout.addWidget(self.MotorPosValue,0,1)

        motorLayout.addWidget(self.MotorCurrSpeedLabel,0,2)
        motorLayout.addWidget(self.MotorCurrSpeedValue,0,3)

        motorLayout.addWidget(self.MotorFasterButton,0,4)
        motorLayout.addWidget(self.MotorSlowerButton,0,5)

        motorLayout.addWidget(self.FastApproachButton,1,0,1,1)
        motorLayout.addWidget(self.SlowApproachButton,2,0,1,1)

        motorLayout.addWidget(self.MotorStopButton,1,1,2,2)

        motorLayout.addWidget(self.FastRetractButton,1,3,1,1)
        motorLayout.addWidget(self.SlowRetractButton,2,3,1,1)

        motorLayout.addWidget(self.FastLimitToggle,1,4,1,1)
        motorLayout.addWidget(self.SlowLimitToggle,2,4,1,1)

        motorLayout.addWidget(self.AutoApproachButton,1,5,2,1)

        buttonHeight = 45

        self.AutoApproachButton.setMinimumHeight(2*buttonHeight)
        self.MotorStopButton.setMinimumHeight(2*buttonHeight)
        self.MotorFasterButton.setMinimumHeight(buttonHeight)
        self.MotorSlowerButton.setMinimumHeight(buttonHeight)
        self.FastApproachButton.setMinimumHeight(buttonHeight)
        self.SlowApproachButton.setMinimumHeight(buttonHeight)
        self.FastRetractButton.setMinimumHeight(buttonHeight)
        self.SlowRetractButton.setMinimumHeight(buttonHeight)
        self.autoApproachTimer = QTimer()
        self.motorCountTimer = QTimer()
        self.motorCountTimer.timeout.connect(self.MotorCount)

        self.SlowLimitToggle.setStyleSheet("QCheckBox::indicator"
                                           "{"
                                           "width: 40px;"
                                           "height: 40px;"
                                           "}")

        self.FastLimitToggle.setStyleSheet("QCheckBox::indicator"
                                           "{"
                                           "width: 40px;"
                                           "height: 40px;"
                                           "}")


        self.MotorBox.setLayout(motorLayout)
        #
        #Motor Control

        layout.addWidget(self.MotorBox,4,0,2,7)

    def SlowLimitCheckFunction(self):
        if self.slowLimitState == 0:
            self.slowLimitState = 1
        else:
            self.slowLimitState = 0

        self.SaveSettings()

    def FastLimitCheckFunction(self):
        if self.fastLimitState == 0:
            self.fastLimitState = 1
        else:
            self.fastLimitState = 0

        self.SaveSettings()


    def SettingsTab(self):
        layout = QtWidgets.QGridLayout(self.settingTab)
        speedLayout = QtWidgets.QGridLayout()
        approachLayout = QtWidgets.QGridLayout()
        advancedLayout = QtWidgets.QGridLayout()

        self.speedBox = QtWidgets.QGroupBox("Speeds")
        layout.addWidget(self.speedBox,0,0)

        self.approachBox = QtWidgets.QGroupBox("Auto Approach Conditions")
        layout.addWidget(self.approachBox,0,1)


        #Speed Settings
        self.AutoApproachSpeedLabel = QtWidgets.QLabel("Auto")
        self.AutoApproachSpeedBox = QtWidgets.QSpinBox()
        self.AutoApproachSpeedBox.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.AutoApproachSpeedBox.setRange(0,60000)
        self.AutoApproachSpeedBox.setValue(self.autoApproachFreq)
        self.AutoAppUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.AutoAppUpButton.setObjectName("AutoAppUpButton")
        self.AutoAppDownButton = QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.AutoAppDownButton.setObjectName("AutoAppDownButton")

        self.SlowSpeedLabel = QtWidgets.QLabel("Slow")
        self.SlowSpeedBox = QtWidgets.QSpinBox()
        self.SlowSpeedBox.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.SlowSpeedBox.setRange(0,60000)
        self.SlowSpeedBox.setValue(self.slowMoveFreq)
        self.SlowSpeedUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.SlowSpeedUpButton.setObjectName("SlowSpeedUpButton")
        self.SlowSpeedDownButton = QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.SlowSpeedDownButton.setObjectName("SlowSpeedDownButton")

        self.FastSpeedLabel = QtWidgets.QLabel("Fast")
        self.FastSpeedBox = QtWidgets.QSpinBox()
        self.FastSpeedBox.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.FastSpeedBox.setRange(0,60000)
        self.FastSpeedBox.setValue(self.fastMoveFreq)
        self.FastSpeedUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.FastSpeedUpButton.setObjectName("FastSpeedUpButton")
        self.FastSpeedDownButton= QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.FastSpeedDownButton.setObjectName("FastSpeedDownButton")

        self.AccelerationLabel = QtWidgets.QLabel("Accel.")
        self.AccelerationBox = QtWidgets.QSpinBox()
        self.AccelerationBox.setButtonSymbols(QtWidgets.QSpinBox.NoButtons)
        self.AccelerationBox.setRange(0,10000)
        self.AccelerationBox.setValue(self.acceleration)
        self.AccelerationUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.AccelerationUpButton.setObjectName("AccelerationUpButton")
        self.AccelerationDownButton = QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.AccelerationDownButton.setObjectName("AccelerationDownButton")

        speedLayout.addWidget(self.AutoApproachSpeedLabel,1,1)
        speedLayout.addWidget(self.AutoApproachSpeedBox,1,2,1,2)
        speedLayout.addWidget(self.AutoAppUpButton,1,4)
        speedLayout.addWidget(self.AutoAppDownButton,1,5)

        speedLayout.addWidget(self.SlowSpeedLabel,2,1)
        speedLayout.addWidget(self.SlowSpeedBox,2,2,1,2)
        speedLayout.addWidget(self.SlowSpeedUpButton,2,4)
        speedLayout.addWidget(self.SlowSpeedDownButton,2,5)

        speedLayout.addWidget(self.FastSpeedLabel,3,1)
        speedLayout.addWidget(self.FastSpeedBox,3,2,1,2)
        speedLayout.addWidget(self.FastSpeedUpButton,3,4)
        speedLayout.addWidget(self.FastSpeedDownButton,3,5)

        speedLayout.addWidget(self.AccelerationLabel,4,1)
        speedLayout.addWidget(self.AccelerationBox,4,2,1,2)
        speedLayout.addWidget(self.AccelerationUpButton,4,4)
        speedLayout.addWidget(self.AccelerationDownButton,4,5)
        self.speedBox.setLayout(speedLayout)

        #Auto Approach Conditions
        self.AmplitudeConditionLabel = QtWidgets.QLabel("Amplitude")
        self.AmplitudeConditionBox = QtWidgets.QSpinBox()
        self.AmplitudeConditionBox.setSuffix("%")
        self.AmplitudeConditionBox.setRange(0,100)
        self.AmplitudeConditionBox.setValue(int(100*self.ampRatio))
        self.AmplitudeUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.AmplitudeUpButton.setObjectName("AmplitudeUpButton")
        self.AmplitudeDownButton = QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.AmplitudeDownButton.setObjectName("AmplitudeDownButton")

        self.DeflectionConditionLabel = QtWidgets.QLabel("Deflection")
        self.DeflectionConditionBox = QtWidgets.QDoubleSpinBox()
        self.DeflectionConditionBox.setSuffix(" V")
        self.DeflectionConditionBox.setRange(-10,10)
        self.DeflectionConditionBox.setValue(self.defLimit)
        self.DeflectionUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.DeflectionUpButton.setObjectName("DeflectionUpButton")
        self.DeflectionDownButton = QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.DeflectionDownButton.setObjectName("DeflectionDownButton")

        self.ZPiezoConditionLabel = QtWidgets.QLabel("z-Piezo")
        self.ZPiezoConditionBox = QtWidgets.QDoubleSpinBox()
        self.ZPiezoConditionBox.setSuffix(" V")
        self.ZPiezoConditionBox.setRange(-10,10)
        self.ZPiezoConditionBox.setValue(self.zpiLimit)
        self.ZPiezoUpButton = QtWidgets.QPushButton("Up", clicked=self.UpDownButtonFunction)
        self.ZPiezoUpButton.setObjectName("ZPiezoUpButton")
        self.ZPiezoDownButton = QtWidgets.QPushButton("Down", clicked=self.UpDownButtonFunction)
        self.ZPiezoDownButton.setObjectName("ZPiezoDownButton")

        approachLayout.addWidget(self.AmplitudeConditionLabel,1,1)
        approachLayout.addWidget(self.AmplitudeConditionBox,1,2,1,2)
        approachLayout.addWidget(self.AmplitudeUpButton,1,4)
        approachLayout.addWidget(self.AmplitudeDownButton,1,5)

        approachLayout.addWidget(self.DeflectionConditionLabel,2,1)
        approachLayout.addWidget(self.DeflectionConditionBox,2,2,1,2)
        approachLayout.addWidget(self.DeflectionUpButton,2,4)
        approachLayout.addWidget(self.DeflectionDownButton,2,5)

        approachLayout.addWidget(self.ZPiezoConditionLabel,3,1)
        approachLayout.addWidget(self.ZPiezoConditionBox,3,2,1,2)
        approachLayout.addWidget(self.ZPiezoUpButton,3,4)
        approachLayout.addWidget(self.ZPiezoDownButton,3,5)

        self.approachBox.setLayout(approachLayout)


    def UpDownButtonFunction(self):
        objectName = self.sender().objectName()
        #Up Buttons
        if objectName == "AutoAppUpButton":
            self.autoApproachFreq += self.dSpeed
        elif objectName  == "SlowSpeedUpButton":
            self.slowMoveFreq += self.dSpeed
        elif objectName == "FastSpeedUpButton":
            self.fastMoveFreq += self.dSpeed
        elif objectName == "AccelerationUpButton":
            self.acceleration += int(self.dSpeed/10)
        elif objectName == "AmplitudeUpButton":
            self.ampRatio += 0.05
        elif objectName == "DeflectionUpButton":
            self.defLimit += 0.2
        elif objectName == "ZPiezoUpButton":
            self.zpiLimit += 0.2
        #Down Buttons
        elif objectName == "AutoAppDownButton":
            self.autoApproachFreq -= self.dSpeed
            if self.autoApproachFreq < 0:
                self.autoApproachFreq = 0
        elif objectName  == "SlowSpeedDownButton":
            self.slowMoveFreq -= self.dSpeed
            if self.slowMoveFreq < 0:
                self.slowMoveFreq = 0
        elif objectName == "FastSpeedDownButton":
            self.fastMoveFreq -= self.dSpeed
            if self.fastMoveFreq < 0:
                self.fastMoveFreq = 0
        elif objectName == "AccelerationDownButton":
            self.acceleration -= int(self.dSpeed/10)
            if self.acceleration < 0:
                self.acceleration = 0
        elif objectName == "AmplitudeDownButton":
            self.ampRatio -= 0.05
            if self.ampRatio < 0:
                self.ampRatio = 0
        elif objectName == "DeflectionDownButton":
            self.defLimit -= 0.2
            if self.defLimit < -10:
                self.defLimit = -10
        elif objectName == "ZPiezoDownButton":
            self.zpiLimit -= 0.2
            if self.zpiLimit < -10:
                self.zpiLimit = -10

        self.AutoApproachSpeedBox.setValue(self.autoApproachFreq)
        self.SlowSpeedBox.setValue(self.slowMoveFreq)
        self.FastSpeedBox.setValue(self.fastMoveFreq)
        self.AccelerationBox.setValue(self.acceleration)

        self.AmplitudeConditionBox.setValue(int(100*self.ampRatio))
        self.DeflectionConditionBox.setValue(self.defLimit)
        self.ZPiezoConditionBox.setValue(self.zpiLimit)
        self.SaveSettings()

    def AdvancedTab(self):
        layout = QtWidgets.QGridLayout(self.advancedTab)

        channelLayout = QtWidgets.QGridLayout()
        advMotorLayout = QtWidgets.QGridLayout()
        advMeterLayout = QtWidgets.QGridLayout()

        self.channelBox = QtWidgets.QGroupBox("Channels")
        self.channelBox.setLayout(channelLayout)
        layout.addWidget(self.channelBox,0,0,5,1)

        self.advMotorBox = QtWidgets.QGroupBox("Advanced Motor Settings")
        self.advMotorBox.setLayout(advMotorLayout)
        layout.addWidget(self.advMotorBox,0,1,3,1)

        self.advMeterBox = QtWidgets.QGroupBox("Advanced Meter Settings")
        self.advMeterBox.setLayout(advMeterLayout)
        layout.addWidget(self.advMeterBox,3,1,2,1)


        #Advanced Settings
        #self.motorSelectLabel = QtWidgets.QLabel("Output")
        #self.motorSelectBox = QtWidgets.QComboBox()
        #self.motorSelectBox.addItem("2 pulses")
        #self.motorSelectBox.addItem("pulse + direction")
        #self.motorSelectBox.setCurrentIndex(self.outputMode)
        #self.motorSelectBox.currentIndexChanged.connect(self.OutputSelectFunction)

        #self.directionChnLabel = QtWidgets.QLabel("Direction Chn.")
        #self.directionChnLabel.setDisabled(True)
        #self.directionChnBox = QtWidgets.QSpinBox()
        #self.directionChnBox.setRange(0,7)
        #self.directionChnBox.setDisabled(True)
        #self.directionChnBox.setObjectName("DirectionChn")
        #self.directionChnBox.setValue(self.directionChn)
        #self.directionChnBox.valueChanged.connect(self.DoAdvancedSettings)

        self.approachChnLabel = QtWidgets.QLabel("Approach Chn.")
        self.approachChnBox = QtWidgets.QSpinBox()
        self.approachChnBox.setRange(2,3)
        self.approachChnBox.setObjectName("ApproachChn")
        self.approachChnBox.setValue(self.approachChn)
        self.approachChnBox.valueChanged.connect(self.DoAdvancedSettings)

        self.retractChnLabel = QtWidgets.QLabel("Retract Chn.")
        self.retractChnBox = QtWidgets.QSpinBox()
        self.retractChnBox.setRange(2,3)
        self.retractChnBox.setObjectName("RetractChn")
        self.retractChnBox.setValue(self.retractChn)
        self.retractChnBox.valueChanged.connect(self.DoAdvancedSettings)

        self.SumChnLabel = QtWidgets.QLabel("Sum Chn.")
        self.SumChnBox = QtWidgets.QSpinBox()
        self.SumChnBox.setRange(0,7)
        self.SumChnBox.setObjectName("SumChn")
        self.SumChnBox.setValue(self.sumChn)
        self.SumChnBox.valueChanged.connect(self.DoAdvancedSettings)

        self.DeflectionChnLabel = QtWidgets.QLabel("Deflection Chn.")
        self.DeflectionChnBox = QtWidgets.QSpinBox()
        self.DeflectionChnBox.setRange(0,7)
        self.DeflectionChnBox.setObjectName("DefChn")
        self.DeflectionChnBox.setValue(self.defChn)
        self.DeflectionChnBox.valueChanged.connect(self.DoAdvancedSettings)

        self.AmplitudeChnLabel = QtWidgets.QLabel("Amplitude Chn.")
        self.AmplitudeChnBox = QtWidgets.QSpinBox()
        self.AmplitudeChnBox.setRange(0,7)
        self.AmplitudeChnBox.setObjectName("AmpChn")
        self.AmplitudeChnBox.setValue(self.ampChn)
        self.AmplitudeChnBox.valueChanged.connect(self.DoAdvancedSettings)

        self.ZPiezoChnLabel = QtWidgets.QLabel("z-Piezo Chn.")
        self.ZPiezoChnBox = QtWidgets.QSpinBox()
        self.ZPiezoChnBox.setRange(0,7)
        self.ZPiezoChnBox.setObjectName("ZChn")
        self.ZPiezoChnBox.setValue(self.zpiChn)
        self.ZPiezoChnBox.valueChanged.connect(self.DoAdvancedSettings)


        self.PowerLabel = QtWidgets.QLabel("Power Cycle")
        self.PowerBox = QtWidgets.QSpinBox()
        self.PowerBox.setRange(0,100)
        self.PowerBox.setObjectName("PowerCycle")
        self.PowerBox.setValue(self.powerCycle)
        self.PowerBox.valueChanged.connect(self.DoAdvancedSettings)
        self.PowerBox.setSuffix("%")

        self.TravelFastLabel = QtWidgets.QLabel("max. travel (fast)")
        self.TravelFastBox = QtWidgets.QSpinBox()
        self.TravelFastBox.setRange(0,1000000000)
        self.TravelFastBox.setObjectName("maxTravFast")
        self.TravelFastBox.setValue(self.maxTravelFast)
        self.TravelFastBox.valueChanged.connect(self.DoAdvancedSettings)

        self.TravelSlowLabel = QtWidgets.QLabel("max. travel (slow)")
        self.TravelSlowBox = QtWidgets.QSpinBox()
        self.TravelSlowBox.setRange(0,100000000)
        self.TravelSlowBox.setObjectName("maxTravSlow")
        self.TravelSlowBox.setValue(self.maxTravelSlow)
        self.TravelSlowBox.valueChanged.connect(self.DoAdvancedSettings)

        self.ADReadIntervalLabel = QtWidgets.QLabel("AD update interval")
        self.ADReadIntervalBox = QtWidgets.QSpinBox()
        self.ADReadIntervalBox.setRange(1,100)
        self.ADReadIntervalBox.setObjectName("ADInterval")
        self.ADReadIntervalBox.setValue(self.ADUpdateTimeMS)
        self.ADReadIntervalBox.valueChanged.connect(self.DoAdvancedSettings)
        self.ADReadIntervalBox.setSuffix(" ms")

        self.GraphUpdateIntervalLabel = QtWidgets.QLabel("Graph update interval")
        self.GraphUpdateIntervalBox = QtWidgets.QSpinBox()
        self.GraphUpdateIntervalBox.setRange(1,250)
        self.GraphUpdateIntervalBox.setObjectName("GraphInterval")
        self.GraphUpdateIntervalBox.setValue(self.graphUpdateTimeMS)
        self.GraphUpdateIntervalBox.valueChanged.connect(self.DoAdvancedSettings)
        self.GraphUpdateIntervalBox.setSuffix(" ms")

        #channelLayout.addWidget(self.motorSelectLabel,1,1)
        #channelLayout.addWidget(self.motorSelectBox,1,2)
        #channelLayout.addWidget(self.directionChnLabel,2,1)
        #channelLayout.addWidget(self.directionChnBox,2,2)
        channelLayout.addWidget(self.approachChnLabel,3,1)
        channelLayout.addWidget(self.approachChnBox,3,2)
        channelLayout.addWidget(self.retractChnLabel,4,1)
        channelLayout.addWidget(self.retractChnBox,4,2)
        channelLayout.addWidget(self.SumChnLabel,5,1)
        channelLayout.addWidget(self.SumChnBox,5,2)
        channelLayout.addWidget(self.DeflectionChnLabel,6,1)
        channelLayout.addWidget(self.DeflectionChnBox,6,2)
        channelLayout.addWidget(self.AmplitudeChnLabel,7,1)
        channelLayout.addWidget(self.AmplitudeChnBox,7,2)
        channelLayout.addWidget(self.ZPiezoChnLabel,8,1)
        channelLayout.addWidget(self.ZPiezoChnBox,8,2)

        advMotorLayout.addWidget(self.PowerLabel,1,1)
        advMotorLayout.addWidget(self.PowerBox,1,2)
        advMotorLayout.addWidget(self.TravelFastLabel,2,1)
        advMotorLayout.addWidget(self.TravelFastBox,2,2)
        advMotorLayout.addWidget(self.TravelSlowLabel,3,1)
        advMotorLayout.addWidget(self.TravelSlowBox,3,2)

        advMeterLayout.addWidget(self.ADReadIntervalLabel,1,1)
        advMeterLayout.addWidget(self.ADReadIntervalBox,1,2)
        advMeterLayout.addWidget(self.GraphUpdateIntervalLabel,2,1)
        advMeterLayout.addWidget(self.GraphUpdateIntervalBox,2,2)

        self.ChangeMotorModeUI(self.outputMode)

    def DoAdvancedSettings(self):
        objectName = self.sender().objectName()

        value = self.sender().value()

        if objectName == "DirectionChn":
            self.directionChn = value
        elif objectName == "ApproachChn":
            self.approachChn = value
            self.pwm_ccw = HardwarePWM(pwm_channel=self.approachChn, hz=self.slowMoveFreq, chip=self.chip)
            self.pwm_cw = HardwarePWM(pwm_channel=self.retractChn, hz=self.slowMoveFreq, chip=self.chip)
        elif objectName == "RetractChn":
            self.retractChn = value
            self.pwm_ccw = HardwarePWM(pwm_channel=self.approachChn, hz=self.slowMoveFreq, chip=self.chip)
            self.pwm_cw = HardwarePWM(pwm_channel=self.retractChn, hz=self.slowMoveFreq, chip=self.chip)
        elif objectName == "SumChn":
            self.sumChn = value
        elif objectName == "DefChn":
            self.defChn = value
        elif objectName == "AmpChn":
            self.ampChn = value
        elif objectName == "ZChn":
            self.zpiChn = value
        elif objectName == "PowerCycle":
            self.powerCycle = value
        elif objectName == "maxTravFast":
            self.maxTravelFast = value
        elif objectName == "maxTravSlow":
            self.maxTravelSlow = value
        elif objectName == "ADInterval":
            self.ADUpdateTimeMS = value
        elif objectName == "GraphInterval":
            self.graphUpdateTimeMS = value

        self.SaveSettings()



    def OutputSelectFunction(self, index):
        self.outputMode = index

        self.ChangeMotorModeUI(self.outputMode)

        self.SaveSettings()


    def ChangeMotorModeUI(self, mode):
        if mode == 1:
            #self.directionChnBox.setDisabled(False)
            #self.directionChnLabel.setDisabled(False)

            #self.approachChnLabel.setText("Pulse Chn.")

            #self.retractChnLabel.setDisabled(True)
            #self.retractChnBox.setDisabled(True)
            pass
        else:
            #self.directionChnBox.setDisabled(True)
            #self.directionChnLabel.setDisabled(True)

            self.approachChnLabel.setText("Approach Chn.")

            self.retractChnLabel.setDisabled(False)
            self.retractChnBox.setDisabled(False)


    def createMenuBar(self):
        self.settingsMenuEntry = QtWidgets.QAction("&Settings", self)


    def ForceTab(self):
        layout = QtWidgets.QGridLayout(self.forceTab)

        self.forceDataPoints = 500
        self.extensionVoltage = -1.0
        self.retractionVoltage = 2.5
        self.contForceFlag  = False
        self.piezoConst = 18.5
        self.gain = 5

        self.ContForceTimer = QTimer()
        self.ContForceTimer.timeout.connect(self.DoForceCurve)

        self.DoForceButton = QtWidgets.QPushButton("Do Force Curve", clicked=self.DoForceCurveButtonFunc)
        self.DoForceButton.setMinimumHeight(40)

        self.DoContForceButton = QtWidgets.QPushButton("Do Continuous Force Curves", clicked=self.DoContForceCurve)
        self.DoContForceButton.setMinimumHeight(40)
        self.DoContForceButton.setMinimumWidth(220)

        self.AutoInvOLSButton = QtWidgets.QPushButton("Auto InvOLS", clicked=self.AutoInvOLSFunc)
        self.AutoInvOLSButton.setMinimumHeight(40)


        self.forceCanvas = FigureCanvas(Figure(figsize=(2,4)))
        self.forceAxes = self.forceCanvas.figure.subplots()
        self.forceCanvas.figure.set_layout_engine('tight')
        self.forceAxes.set_xlabel("z-piezo / nm")
        self.forceAxes.set_ylabel("deflection / V")
        self.forceLine, = self.forceAxes.plot([0,1],[0,0],'r')

        self.forcePointsLabel = QtWidgets.QLabel("Data points:")
        self.forcePointsValue = QtWidgets.QSpinBox()
        self.forcePointsValue.setMaximum(1000000)
        self.forcePointsValue.setValue(self.forceDataPoints)
        self.forcePointsValue.valueChanged.connect(self.DoForceSettings)

        self.forceMaxDistLabel = QtWidgets.QLabel("Max. extension:")
        self.forceMaxDistValue = QtWidgets.QDoubleSpinBox()
        self.forceMaxDistValue.setSuffix(" V")
        self.forceMaxDistValue.setMaximum(2.5)
        self.forceMaxDistValue.setMinimum(-2.5)
        self.forceMaxDistValue.setSingleStep(0.1)
        self.forceMaxDistValue.setValue(self.extensionVoltage)
        self.forceMaxDistValue.valueChanged.connect(self.DoForceSettings)

        self.forceMinDistLabel = QtWidgets.QLabel("Max. retraction:")
        self.forceMinDistValue = QtWidgets.QDoubleSpinBox()
        self.forceMinDistValue.setSuffix(" V")
        self.forceMinDistValue.setMaximum(2.5)
        self.forceMinDistValue.setMinimum(-2.5)
        self.forceMinDistValue.setSingleStep(0.1)
        self.forceMinDistValue.setValue(self.retractionVoltage)
        self.forceMinDistValue.valueChanged.connect(self.DoForceSettings)


        self.piezoConstLabel = QtWidgets.QLabel("Piezo const.:")
        self.piezoConstValue = QtWidgets.QDoubleSpinBox()
        self.piezoConstValue.setSuffix(" nm/V")
        self.piezoConstValue.setMaximum(100)
        self.piezoConstValue.setSingleStep(0.1)
        self.piezoConstValue.setValue(self.piezoConst)
        self.piezoConstValue.valueChanged.connect(self.DoForceSettings)

        self.gainLabel = QtWidgets.QLabel("Gain:")
        self.gainValue = QtWidgets.QSpinBox()
        self.gainValue.setMaximum(10)
        self.gainValue.setSingleStep(1)
        self.gainValue.setValue(self.gain)
        self.gainValue.valueChanged.connect(self.DoForceSettings)




        layout.addWidget(self.forceCanvas,0,0,7,3)

        layout.addWidget(self.piezoConstLabel,0,3)
        layout.addWidget(self.piezoConstValue,0,4)

        layout.addWidget(self.gainLabel,1,3)
        layout.addWidget(self.gainValue,1,4)

        layout.addWidget(self.forcePointsLabel,2,3)
        layout.addWidget(self.forcePointsValue,2,4)


        layout.addWidget(self.forceMinDistLabel,3,3)
        layout.addWidget(self.forceMinDistValue,3,4)

        layout.addWidget(self.forceMaxDistLabel,4,3)
        layout.addWidget(self.forceMaxDistValue,4,4)

        layout.addWidget(self.AutoInvOLSButton,5,5)

        layout.addWidget(self.DoForceButton,6,5)
        layout.addWidget(self.DoContForceButton,6,3,1,2)


    def DoZeroEstimate(self):
        N = len(self.ForceDistMRet)
        x1 = np.array(self.ForceDistMRet[int(0.9*N):N-1])
        y1 = np.array(self.ForceDeflDataRet[int(0.9*N):N-1])
        x2 = np.array(self.ForceDistMRet[0:int(0.1*N)])
        y2 = np.array(self.ForceDeflDataRet[0:int(0.1*N)])

        C1,_ = self.DoPolyFit(x1,y1,1)
        C2,_ = self.DoPolyFit(x2,y2,1)

        self.x0 = (C2[1] - C1[1])/(C1[0] - C2[0])
        self.N0 = int(N*(self.x0 - self.ForceDistMRet[0])/(self.ForceDistMRet[N-1] - self.ForceDistMRet[0]))
        self.ForceDistMRet -= self.x0
        self.ForceDistMApp -= self.x0


    def AutoInvOLSFunc(self):
        #self.ForceDistMRet
        #self.ForceDeflDataRet
        N_fit = int(0.8*self.N0)
        x = np.array(self.ForceDistMRet[0:N_fit])
        y = np.array(self.ForceDeflDataRet[0:N_fit])

        C,_ = self.DoPolyFit(x,y,1)
        self.InvOLS = 1/C[0,0]
        print(self.InvOLS)




    def DoPolyFit(self,x,y,deg):
        #Linear Fitting Function
        A = np.c_[np.ones(x.shape[0])]
        xT = np.atleast_2d(x).T

        for i in range(1,deg+1):
            A = np.c_[xT**i,A]

        C,_,_,_ = np.linalg.lstsq(A,np.atleast_2d(y).T,rcond=None)

        yfit = np.matmul(A,C)

        return C,yfit

    def DoForceSettings(self):
        self.forceDataPoints = self.forcePointsValue.value()
        self.extensionVoltage = self.forceMaxDistValue.value()
        self.retractionVoltage = self.forceMinDistValue.value()
        self.piezoConst = self.piezoConstValue.value()
        self.gain = self.gainValue.value()

    def DoForceCurveButtonFunc(self):
        self.ReadADTimer.stop()
        self.DoForceCurve()
        self.ReadADTimer.start(self.ADUpdateTimeMS)


    def DoForceCurve(self):

        N = int(self.forceDataPoints/2)
        V = self.retractionVoltage - self.extensionVoltage
        self.ForceDeflDataApp = array.array('f',[])
        self.ForceDistDataApp = array.array('f',[])
        self.ForceDeflDataRet = array.array('f',[])
        self.ForceDistDataRet = array.array('f',[])


        dV = V/N
        retractPnts = 100

        for i in range(0,retractPnts):
            #voltage = self.forceOffset + i*self.retractionVoltage/retractPnts
            voltage = self.forceOffset + self.retractionVoltage*(1-math.cos(i*math.pi/retractPnts))/2
            self.DAHat.hat.a_out_write(0,voltage)

        F0 = self.forceOffset + self.retractionVoltage

        for i in range(0,N):
            self.DAHat.hat.a_out_write(0,F0 - i*dV)
            self.ForceDeflDataApp.append(self.ADHat.hat.a_in_read(self.defChn,self.ADHat.options))
            self.ForceDistDataApp.append(self.ADHat.hat.a_in_read(4,self.ADHat.options))

        for i in range(0,N):
            self.DAHat.hat.a_out_write(0,F0 - (N-i)*dV)
            self.ForceDeflDataRet.append(self.ADHat.hat.a_in_read(self.defChn,self.ADHat.options))
            self.ForceDistDataRet.append(self.ADHat.hat.a_in_read(4,self.ADHat.options))

        for i in range(0,retractPnts):
            voltage = self.forceOffset + self.retractionVoltage*(1+math.cos(i*math.pi/retractPnts))/2
            self.DAHat.hat.a_out_write(0,voltage)

        dummy = [(x-self.forceOffset)*self.piezoConst*self.gain for x in self.ForceDistDataApp]
        self.ForceDistMApp = array.array('f',dummy)

        dummy2 = [(x-self.forceOffset)*self.piezoConst*self.gain for x in self.ForceDistDataRet]
        self.ForceDistMRet = array.array('f',dummy2)

        #self.forceDistMRet = self.ForceDeflDataApp*self.piezoConst*self.gain
        self.DoZeroEstimate()

        self.forceAxes.cla()
        self.forceLine, = self.forceAxes.plot(self.ForceDistMApp,self.ForceDeflDataApp,'r')
        self.forceLine, = self.forceAxes.plot(self.ForceDistMRet,self.ForceDeflDataRet,'b')
        self.forceCanvas.draw()

    def DoContForceCurve(self):
        self.DoForceButton.setEnabled(0)
        self.DoContForceButton.setText("Stop!")
        self.DoContForceButton.clicked.connect(self.StopContForceCurve)

        self.ContForceTimer.start()

    def StopContForceCurve(self):
        #self.contForceFlag = False
        self.DoForceButton.setEnabled(1)
        self.DoContForceButton.setText("Do Continuous Force Curves")
        self.DoContForceButton.clicked.connect(self.DoContForceCurve)
        self.ContForceTimer.stop()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.MotorStop()
        self.ReadADTimer.stop()

        super(MainWindow, self).closeEvent(event)

    def MeterStopButtonFunction(self):
        #if the system wide color scheme was changed, pressing this button will set the bars 
        #background color to match the system's style
        self.bg_color = self.centralFrame.palette().color(QtGui.QPalette.Window).name()
        self.canvas.figure.set_facecolor(self.bg_color)

        if self.meterRunning:
            #self.ReadADTimer.stop()
            self.meterRunning = False
            self.MeterStopButton.setText("Start\n Meter")
        else:
            #self.ReadADTimer.start(self.ADUpdateTimeMS)
            self.meterRunning = True
            self.MeterStopButton.setText("Stop\n Meter")



    def updateADTimer(self):
        self.sumV = self.ADHat.hat.a_in_read(self.sumChn,self.ADHat.options)
        self.defV = self.ADHat.hat.a_in_read(self.defChn,self.ADHat.options)
        self.ampV = self.ADHat.hat.a_in_read(self.ampChn,self.ADHat.options)
        self.zpiV = self.ADHat.hat.a_in_read(self.zpiChn,self.ADHat.options)

        #Check the stopping conditions during Auto Approach
        if (self.motorRunning == True) and (self.autoApproach == True):
            self.AutoApproachCheck()

        if self.meterRunning:

            self.currGraphCount += 1
            #Don't update the graph and numbers every time, it is too costly
            if self.currGraphCount*self.ADUpdateTimeMS >= self.graphUpdateTimeMS:
                self.sumValue.setValue(self.sumV)
                self.deflectionValue.setValue(self.defV)
                self.amplitudeValue.setValue(self.ampV)
                self.zPiezoValue.setValue(self.zpiV)
                self.setHBarPlot(self.sumV,self.defV,self.zpiV,self.ampV)
                self.currGraphCount = 0

                gc.collect(generation=2)


    def setHBarPlot(self,x,y,z,a):

        self.sumRect.set_width(10*x)

        self.defRect.set_width(y)
        if (y < 0):
            self.defRect.set_color(self.defColor1)
        else:
            self.defRect.set_color(self.defColor2)

        self.ampRect.set_width(10*a)


        self.zpiRect.set_width(z)
        if (z < 0):
            self.zpiRect.set_color(self.zpiColor1)
        else:
            self.zpiRect.set_color(self.zpiColor2)

        self.canvas.restore_region(self.FigBG)
        self.axes.draw_artist(self.sumPatch)
        self.axes.draw_artist(self.defPatch)
        self.axes.draw_artist(self.ampPatch)
        self.axes.draw_artist(self.zpiPatch)

        self.canvas.blit(self.canvas.figure.bbox)

        #self.canvas.draw()
        self.canvas.flush_events()
        #gc.collect()

    def MotorCount(self):
        if self.pulseFreq < 10000:
            dStep = int(1e-3*self.motorUpdateTimeMS*self.pulseFreq/1e1)
            self.motorPos += self.motorDirection*dStep
            self.MotorPosValue.setText(str(self.motorPos))

            if (self.slowLimitState != 0) and (self.slowTravel == 1):
                self.motorDist = abs(self.motorPos - self.startPos)
                if self.motorDist >= self.maxTravelSlow:
                    self.MotorStop()

            if (self.fastLimitState != 0) and (self.fastTravel == 1):
                self.motorDist = abs(self.motorPos - self.startPos)
                if self.motorDist >= self.maxTravelFast:
                    self.MotorStop()
        else:
            self.MotorCountAccel()

    def MotorCountAccel(self):
        dStep = int(1e-3*self.motorUpdateTimeMS*self.accelFreq/1e1)
        self.motorPos += self.motorDirection*dStep
        #self.MotorPosValue.setValue(self.motorPos)
        self.MotorPosValue.setText(str(self.motorPos))

        if (self.slowLimitState != 0) and (self.slowTravel == 1):
            self.motorDist = abs(self.motorPos - self.startPos)
            if self.motorDist >= self.maxTravelSlow:
                self.MotorStop()

        if (self.fastLimitState != 0) and (self.fastTravel == 1):
            self.motorDist = abs(self.motorPos - self.startPos)
            if self.motorDist >= self.maxTravelFast:
                self.MotorStop()


    def MotorStart(self):

        if self.pulseFreq > 10000:
            #self.motorCountTimer.timeout.connect(self.MotorCountAccel)
            self.accelFreq = 0
            finalFreq = self.pulseFreq
            self.AccelTimer.start(self.AccelTimeMS)
            #self.pulseFreq = finalFreq
        else:
            #self.motorCountTimer.timeout.connect(self.MotorCount)
            if self.motorDirection == 1:
                #Retract
                if self.outputMode == 0:
                    self.pwm_cw.change_frequency(self.pulseFreq)
                    self.pwm_cw.start(self.powerCycle)
                else:
                    print(self.directionChn)
                    self.DAHat.hat.dio_output_write_bit(self.directionChn,1)
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)

            elif self.motorDirection == -1:
                #Approach
                if self.outputMode == 0:
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)
                else:
                    print(self.directionChn)
                    self.DAHat.hat.dio_output_write_bit(self.directionChn,0)
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)

            #self.MotorCurrSpeedValue.setValue(self.pulseFreq)
            self.MotorCurrSpeedValue.setText(str(self.pulseFreq))

        self.motorRunning = True
        self.motorCountTimer.start(self.motorUpdateTimeMS)

    def SlowRetractButtonFunction(self):
        self.pulseFreq = self.slowMoveFreq
        self.motorDirection = 1
        self.startPos = self.motorPos
        self.slowTravel = 1
        self.fastTravel = 0
        self.MotorStart()


    def SlowApproachButtonFunction(self):
        self.pulseFreq = self.slowMoveFreq
        self.motorDirection = -1
        self.startPos = self.motorPos
        self.slowTravel = 1
        self.fastTravel = 0
        self.MotorStart()


    def FastRetractButtonFunction(self):
        self.pulseFreq = self.fastMoveFreq
        self.motorDirection = 1
        self.startPos = self.motorPos
        self.slowTravel = 0
        self.fastTravel = 1
        self.MotorStart()


    def FastApproachButtonFunction(self):
        self.pulseFreq = self.fastMoveFreq
        self.motorDirection = -1
        self.startPos = self.motorPos
        self.slowTravel = 0
        self.fastTravel = 1
        self.MotorStart()

    def AutoApproachButtonFunction(self):
        self.pulseFreq = self.autoApproachFreq
        self.motorDirection = -1
        self.autoApproach = True
        self.initialAmp = self.ampV
        self.MotorStart()

    def AutoApproachCheck(self):
        if (self.ampV < self.initialAmp*self.ampRatio) or (self.zpiV < self.zpiLimit) or (self.defV > self.defLimit):
            self.MotorStop()

    def MotorStopButtonFunction(self):
        self.MotorStop()

    def MotorStop(self):
        self.AccelTimer.stop()
        self.pwm_cw.stop()
        self.pwm_ccw.stop()
        self.motorCountTimer.stop()
        self.motorRunning = False
        self.autoApproach = False
        self.pulseFreq = 0
        self.slowTravel = 0
        self.fastTravel = 0
        #self.MotorCurrSpeedValue.setValue(self.pulseFreq)
        self.MotorCurrSpeedValue.setText(str(self.pulseFreq))

    def FasterButtonFunction(self):
        if self.motorRunning == True:
            #self.pwm_cw.stop()
            #self.pwm_cw.stop()
            self.motorCountTimer.stop()
            self.pulseFreq += self.dSpeed

            #self.MotorStart()
            #self.motorCountTimer.timeout.connect(self.MotorCount)
            if self.motorDirection == 1:
                #Retract
                if self.outputMode == 0:
                    self.pwm_cw.change_frequency(self.pulseFreq)
                    self.pwm_cw.start(self.powerCycle)
                else:
                    print(self.directionChn)
                    self.DAHat.hat.dio_output_write_bit(self.directionChn,1)
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)

            elif self.motorDirection == -1:
                #Approach
                if self.outputMode == 0:
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)
                else:
                    print(self.directionChn)
                    self.DAHat.hat.dio_output_write_bit(self.directionChn,0)
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)

            #self.MotorCurrSpeedValue.setValue(self.pulseFreq)
            self.MotorCurrSpeedValue.setText(str(self.pulseFreq))

            self.motorRunning = True
            self.motorCountTimer.start(self.motorUpdateTimeMS)



    def SlowerButtonFunction(self):
        if self.motorRunning == True:
            #self.pwm_ccw.stop()
            #self.pwm_cw.stop()
            self.motorCountTimer.stop()
            self.pulseFreq -= self.dSpeed
            #The minimum frequency is 0.1, just in case, let's keep it 1 at the lowest 
            if self.pulseFreq < 1:
                self.pulseFreq = 1
            #self.MotorStart()
            #self.motorCountTimer.timeout.connect(self.MotorCount)
            if self.motorDirection == 1:
                #Retract
                if self.outputMode == 0:
                    self.pwm_cw.change_frequency(self.pulseFreq)
                    self.pwm_cw.start(self.powerCycle)
                else:
                    print(self.directionChn)
                    self.DAHat.hat.dio_output_write_bit(self.directionChn,1)
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)

            elif self.motorDirection == -1:
                #Approach
                if self.outputMode == 0:
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)
                else:
                    print(self.directionChn)
                    self.DAHat.hat.dio_output_write_bit(self.directionChn,0)
                    self.pwm_ccw.change_frequency(self.pulseFreq)
                    self.pwm_ccw.start(self.powerCycle)

            #self.MotorCurrSpeedValue.setValue(self.pulseFreq)
            self.MotorCurrSpeedValue.setText(str(self.pulseFreq))

            self.motorRunning = True
            self.motorCountTimer.start(self.motorUpdateTimeMS)



    def AcceleratedMovement(self):
        self.accelFreq += self.acceleration
        if self.motorDirection == 1:
            #Retract
            if self.outputMode == 0:
                self.pwm_cw.change_frequency(self.accelFreq)
                self.pwm_cw.start(self.powerCycle)
        elif self.motorDirection == -1:
            #Approach
            if self.outputMode == 0:
                self.pwm_ccw.change_frequency(self.accelFreq)
                self.pwm_ccw.start(self.powerCycle)

        #self.MotorCurrSpeedValue.setValue(self.accelFreq)
        self.MotorCurrSpeedValue.setText(str(self.pulseFreq))

        if self.accelFreq >= self.pulseFreq:
            self.AccelTimer.stop()





    def float_to_bytes(self,value):
        return struct.pack('d', value)

    def bytes_to_float(self,value):
        [result] = struct.unpack('d', value)
        return  result

    def SaveSettings(self):
        settings_file = open("settings.dat","wb")

        settings_file.write(self.ADUpdateTimeMS.to_bytes(8,byteorder='big'))
        settings_file.write(self.graphUpdateTimeMS.to_bytes(8,byteorder='big'))
        settings_file.write(self.currGraphCount.to_bytes(8,byteorder='big'))
        settings_file.write(self.fastMoveFreq.to_bytes(8,byteorder='big'))
        settings_file.write(self.slowMoveFreq.to_bytes(8,byteorder='big'))
        settings_file.write(self.autoApproachFreq.to_bytes(8,byteorder='big'))
        settings_file.write(self.acceleration.to_bytes(8,byteorder='big'))
        settings_file.write(self.dSpeed.to_bytes(8,byteorder='big'))
        settings_file.write(self.powerCycle.to_bytes(8,byteorder='big'))
        settings_file.write(self.accelFreq.to_bytes(8,byteorder='big'))
        settings_file.write(self.approachChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.retractChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.sumChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.defChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.ampChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.zpiChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.float_to_bytes(self.defLimit))
        settings_file.write(self.float_to_bytes(self.zpiLimit))
        settings_file.write(self.float_to_bytes(self.ampRatio))
        settings_file.write(self.motorUpdateTimeMS.to_bytes(8,byteorder='big'))
        settings_file.write(self.outputMode.to_bytes(8,byteorder='big'))

        settings_file.write(self.ADUpdateTimeMS.to_bytes(8,byteorder='big'))
        settings_file.write(self.graphUpdateTimeMS.to_bytes(8,byteorder='big'))
        settings_file.write(self.maxTravelFast.to_bytes(8,byteorder='big'))
        settings_file.write(self.maxTravelSlow.to_bytes(8,byteorder='big'))
        settings_file.write(self.directionChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.approachChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.retractChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.sumChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.defChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.ampChn.to_bytes(8,byteorder='big'))
        settings_file.write(self.zpiChn.to_bytes(8,byteorder='big'))

        settings_file.write(self.fastLimitState.to_bytes(8,byteorder='big'))
        settings_file.write(self.slowLimitState.to_bytes(8,byteorder='big'))

        settings_file.close()

        #self.LoadSettings()

    def LoadSettings(self):
        try:
            settings_file = open("settings.dat", "rb")

            self.ADUpdateTimeMS = int.from_bytes(settings_file.read(8),byteorder='big')
            self.graphUpdateTimeMS = int.from_bytes(settings_file.read(8),byteorder='big')
            self.currGraphCount = int.from_bytes(settings_file.read(8),byteorder='big')
            self.fastMoveFreq = int.from_bytes(settings_file.read(8),byteorder='big')
            self.slowMoveFreq = int.from_bytes(settings_file.read(8),byteorder='big')
            self.autoApproachFreq = int.from_bytes(settings_file.read(8),byteorder='big')
            self.acceleration = int.from_bytes(settings_file.read(8),byteorder='big')
            self.dSpeed = int.from_bytes(settings_file.read(8),byteorder='big')
            self.powerCycle = int.from_bytes(settings_file.read(8),byteorder='big')
            self.accelFreq = int.from_bytes(settings_file.read(8),byteorder='big')
            self.approachChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.retractChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.sumChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.defChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.ampChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.zpiChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.defLimit = self.bytes_to_float(settings_file.read(8))
            self.zpiLimit = self.bytes_to_float(settings_file.read(8))
            self.ampRatio = self.bytes_to_float(settings_file.read(8))
            dummy = int.from_bytes(settings_file.read(8),byteorder='big')
            self.outputMode = int.from_bytes(settings_file.read(8),byteorder='big')

            self.ADUpdateTimeMS = int.from_bytes(settings_file.read(8),byteorder='big')
            self.graphUpdateTimeMS = int.from_bytes(settings_file.read(8),byteorder='big')
            self.maxTravelFast = int.from_bytes(settings_file.read(8),byteorder='big')
            self.maxTravelSlow = int.from_bytes(settings_file.read(8),byteorder='big')
            self.directionChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.approachChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.retrachChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.sumChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.defChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.ampChn = int.from_bytes(settings_file.read(8),byteorder='big')
            self.zpiChn = int.from_bytes(settings_file.read(8),byteorder='big')

            self.fastLimitState = int.from_bytes(settings_file.read(8),byteorder='big')
            self.slowLimitState = int.from_bytes(settings_file.read(8),byteorder='big')

            settings_file.close()
        except:
            print("Settings file 'settings.dat' not found; creating one for next time.")
            self.SaveSettings()






if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    widget = MainWindow()
    widget.show()

    sys.exit(app.exec())
