
import sys
import os

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

import serial
import binascii
import re
import queue
import threading
import math
import time
import sys
# from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5 import *
from PyQt5 import uic
import serial.tools.list_ports  # For listing available serial ports
import numpy as np
import random




## HELLO
form = resource_path("Lidar_final.ui")
form_class = uic.loadUiType(form)[0]
# form_class = uic.loadUiType("Lidar3.ui")[0]
__platform__ = sys.platform


# form_class = uic.loadUiType("Lidar_final.ui")[0]
# __platform__ = sys.platform

q_box = queue.Queue()
n = 0  ##star, stop index
angle = 0
wait_flag = 0
lidar_start = 1

Pos_flag = 0    # Indicate the position of the quadrant

DataPos0 = 0    # Distance between quadrant 1 data
DataPos1 = 0    # Distance between quadrant 2 data
DataPos2 = 0    # Distance between quadrant 3 data
DataPos3 = 0    # Distance between quadrant 4 data

hexStr = [] ## Hex data conversion list


ToFPosDataQ0 = []   # Quadrant 1 Distance List
ToFPosDataQ1 = []   # Quadrant 2 Distance List
ToFPosDataQ2 = []   # Quadrant 3 Distance List
ToFPosDataQ3 = []   # Quadrant 4 Distance List

COM = 'COM'
com_start = 0
get_data_thr = 1
serial_start = 0
write = 0
# dev = serial.tools.list_ports.comports()
#
# global ports
# ports = []
#
# for d in dev:
#     ports.append((d.device))
#
# ports.sort()
# print(ports)
# COM = random.choice(ports)
#


class Window(QMainWindow,form_class): #, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("NanoLidar 0.2")
        self.setFixedSize(800, 650)   ## fixed size

        self.lbl = QLabel(self)
        # self.lbl.resize(300,200)


        # self.robo_vertical.addWidget(self.lbl)

        self.label.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        self.label.setStyleSheet("background-color : #e1e1e1")
        self.label_3.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        self.label_3.setStyleSheet("background-color : #e1e1e1")
        self.label_5.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        self.label_5.setStyleSheet("background-color : #e1e1e1")
        self.label_7.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        self.label_7.setStyleSheet("background-color : #e1e1e1")
        self.label_2.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        self.label_2.setStyleSheet("background-color : #e1e1e1")
        self.label_9.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        self.label_9.setStyleSheet("background-color : #e1e1e1")
        # self.label.setFont(QtGui.QFont("Yu Gothic UI Semibold", 10))
        # self.label.setStyleSheet("background-color : #e2e2e2")



        # self.label.setStyleSheet("background-color : #bfbfbf;")# "border-style : dashed;" "boder-width : 2px" "border-color : #FA8072;" "border-radius : 3px")
        # self.label.setStyleSheet("background-color : #bfbfbf;" "border-style : solid;" "border-width : 2px;" "border-color : #FA8072;" "border-radius : 3px;")
        # self.gridLayout.setStyleSheet("border-style : solid;" "border-width : 2px;" "border-color : #FA8072;" "border-radius : 3px;")
        # self.groupBox.setStyleSheet( "border-style : solid;" "border-width : 1px;")
        # self.groupBox_2.setStyleSheet( "border-style : solid;" "border-width : 1px;")
        # self.lbl2 = QLabel(self)
        # self.lbl2.resize(300, 200)
        # pixmap2 = QtGui.QPixmap("rb_logo_200px.png")
        # pixmap2 = pixmap2.scaledToWidth(150)
        # self.lbl2.setPixmap(QtGui.QPixmap(pixmap2))
        ########################################
        self.NanoLidar.setStyleSheet("border-image: url(Nano Lidar2x.png);")
        self.start_btn.setStyleSheet("border-image: url(Start2x.png);")
        self.stop_btn.setStyleSheet("border-image: url(Stop2x.png);")
        self.serial_btn_2.setStyleSheet("border-image: url(Open.png);")
        self.serial_btn.setStyleSheet("border-image: url(close.png);")
        self.run_state_txt.setStyleSheet("border-image: url(Stop_02.png);")
        self.title.setStyleSheet("color : white;" "background-color : black;")
        self.COM.setStyleSheet("background-color : white;" "border-color : gray;" "border-style : solid;" "border-width : 0.5px;")


        self.label_3.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.label_5.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.label_7.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.label_2.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.label.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")
        self.label_9.setStyleSheet(
            "background-color : #f5f6fb;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 2px;")

        self.index_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.systemstate_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.synchronous_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.angle_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.distance_length_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        self.output_txt.setStyleSheet(
            "background-color : white;" "border-color : #e0dee1;" "border-style : solid;" "border-width : 0.5px;"  "border-left-width : 0.5px;")
        ########################################



        # self.lidar_graph.adjustSize()
        self.lidar_graph.disableAutoRange()
        # self.lidar_graph.enableAutoRange(axis="x")
        # self.lidar_graph.setMouseEnabled(x=True, y=True)
        self.lidar_graph.setMouseEnabled(x=False,y=False)


        # self.run_state_txt.setText("STOP")
        # self.run_state_txt.setFont(QtGui.QFont("Yu Gothic UI Semibold", 20))
        # self.run_state_txt.setStyleSheet("Color : red")



        self.lidar_graph.setXRange(-550,550)
        self.lidar_graph.setYRange(-550,550)
        self.lidar_graph.setMouseEnabled()
        # self.lidar_graph.setTitle(title="Lidar GUI(mm)")
        self.lidar_graph.addLine(x=True)
        self.lidar_graph.addLine(y=True)
        # self.lidar_graph.showGrid(x=True, y=True)
        self.lidar_graph.setLabel('top', text='0°')
        self.lidar_graph.setLabel('bottom', text='180°')
        self.lidar_graph.setLabel('right', text='90°')
        self.lidar_graph.setLabel('left', text='240°')
        self.lidar_graph.setBackground('w')
        # self.robomation_png.scaledToWidth(10)

        self.theta = np.linspace(0, 2 * np.pi, 100)
        # self.radius = 1000
        self.a1 = 100 * np.cos(self.theta)
        self.b1 = 100 * np.sin(self.theta)
        self.a2 = 500 * np.cos(self.theta)
        self.b2 = 500 * np.sin(self.theta)
        self.a3 = 1200 * np.cos(self.theta)
        self.b3 = 1200 * np.sin(self.theta)
        self.a4 = 2000 * np.cos(self.theta)
        self.b4 = 2000 * np.sin(self.theta)
        self.a5 = 3000 * np.cos(self.theta)
        self.b5 = 3000 * np.sin(self.theta)
        self.a6 = 4000 * np.cos(self.theta)
        self.b6 = 4000 * np.sin(self.theta)
        self.a7 = 5000 * np.cos(self.theta)
        self.b7 = 5000 * np.sin(self.theta)
        self.a8 = 6000 * np.cos(self.theta)
        self.b8 = 6000 * np.sin(self.theta)
        self.a9 = 7000 * np.cos(self.theta)
        self.b9 = 7000 * np.sin(self.theta)
        self.a10 = 8000 * np.cos(self.theta)
        self.b10 = 8000 * np.sin(self.theta)
        self.a11 = 9000 * np.cos(self.theta)
        self.b11 = 9000 * np.sin(self.theta)
        self.a12 = 10000 * np.cos(self.theta)
        self.b12 = 10000 * np.sin(self.theta)
        self.a13 = 11000 * np.cos(self.theta)
        self.b13 = 11000 * np.sin(self.theta)



        self.a14 = [-8000, 8000]
        self.b14 = [13856, -13856]
        self.a15 = [-8000, 8000]
        self.b15 = [4619, -4619]
        self.a16 = [-8000, 8000]
        self.b16 = [-13856, 13856]
        self.a17 = [-8000, 8000]
        self.b17 = [-4619, 4619]


        self.lidar_graph.plot(self.a1, self.b1)
        self.lidar_graph.plot(self.a2, self.b2)
        self.lidar_graph.plot(self.a3, self.b3)
        self.lidar_graph.plot(self.a4, self.b4)
        self.lidar_graph.plot(self.a5, self.b5)
        self.lidar_graph.plot(self.a6, self.b6)
        self.lidar_graph.plot(self.a7, self.b7)
        self.lidar_graph.plot(self.a8, self.b8)
        self.lidar_graph.plot(self.a9, self.b9)
        self.lidar_graph.plot(self.a10, self.b10)
        self.lidar_graph.plot(self.a11, self.b11)
        self.lidar_graph.plot(self.a12, self.b12)
        self.lidar_graph.plot(self.a13, self.b13)
        self.lidar_graph.plot(self.a14, self.b14)
        self.lidar_graph.plot(self.a15, self.b15)
        self.lidar_graph.plot(self.a16, self.b16)
        self.lidar_graph.plot(self.a17, self.b17)


        self.curve = self.lidar_graph.plot(pen=None, symbol='o',symbolsize=0.001)
        self.curve.setSymbolSize(6)
        # self.curve = self.lidar_graph.plot(pen=None, symbol='+', symbolBrush=1)
        self.angle_txt.setText(str(angle))
        # print(angle)
        #############################################
        menubar = self.menuBar()
        filemenu = menubar.addMenu("&PORT SELECT")

        dev = serial.tools.list_ports.comports()

        global ports
        ports = []

        for d in dev:
            ports.append((d.device, d.serial_number))

        ports.sort()
        print(ports)

        for d in range(len(ports)):
            globals()[ports[d][0]] = str(ports[d][0])
            # print(globals()[ports[d][0]])

            # self.COM = globals()[ports[d][0]]
            # globals()['COM{}_selct'.format(self.COM[3])] = globals()[ports[d][0]]

            globals()[ports[d][0]] = QAction(str(ports[d][0]), self)
            filemenu.addAction(globals()[ports[d][0]])

            globals()[ports[d][0]].triggered.connect(self.Port_num)

        self.start_btn.pressed.connect(self.start_btn_pressed)
        self.stop_btn.pressed.connect(self.stop_btn_pressed)

    def start_btn_pressed(self):
        self.start_btn.setStyleSheet("border-image: url(Start_in2x.png);")
    def stop_btn_pressed(self):
        self.stop_btn.setStyleSheet("border-image: url(Stop_in2x.png);")



    def Port_num(self,state):
        global COM, com_start
        action = self.sender()

        for i in range(len(ports)):
            if action.text() == ports[i][0]:
                print(action.text())
                COM = action.text()
                self.COM.setText(COM)



        # self.Send_Stop_Data()


    def serial_start(self):
        global COM, com_start, serial_start
        try:
            self.ser = serial.Serial(
                port=COM,
                baudrate=115200,
                # baudrate=460800,
                # 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )


            self.serial_btn_2.setStyleSheet("border-image: url(Open.png);")
            self.serial_btn.setStyleSheet("border-image: url(close_in.png);")
            print("Serial Port Select")
            print("COM number : ", COM)
            com_start = 1
            serial_start = 1
            create_thread()
        except serial.serialutil.SerialException:
            print("No comfort specified or Comfort is already open")


        # Serial()
        # create_thread()
        # self.Send_Stop_Data()

    def serial_stop(self):
        global com_start, serial_start
        try:
            com_start = 0
            serial_start = 0
            self.ser.close()

            self.serial_btn_2.setStyleSheet("border-image: url(Open_in.png);")
            self.serial_btn.setStyleSheet("border-image: url(close.png);")
            print("Serial Port Cancel")
            # Serial().ser.close()

            time.sleep(0.1)
        except AttributeError:
            print("Check the comfort connection or Comfort is already open.")
            time.sleep(0.1)




    def update(self):
        self.curve.setData(self.totalToFPosX, self.totalToFPosY)
        time.sleep(0.001)
    # def clear_graph(self):
    #     self.curve.clear()

    def show_data(self):
        self.ToFPosX0 = []
        self.ToFPosY0 = []

        self.ToFPosX1 = []
        self.ToFPosY1 = []

        self.ToFPosX2 = []
        self.ToFPosY2 = []

        self.ToFPosX3 = []
        self.ToFPosY3 = []

        self.totalToFPosX = []
        self.totalToFPosY = []

        # self.clear_graph()

        try:

            for i in range(DataPos0):
                if ToFPosDataQ0[i] != 0xffff:
                    self.ToFPosX0.append(ToFPosDataQ0[i] * math.cos(-math.radians(((90 * i) / DataPos0)-90)))
                    self.ToFPosY0.append(ToFPosDataQ0[i] * math.sin(-math.radians(((90 * i) / DataPos0)-90)))
                # else:
                #     self.ToFPosX0.append()
                #     self.ToFPosY0.append()
            for i in range(DataPos1):
                if ToFPosDataQ1[i] != 0xffff:
                    self.ToFPosX1.append(ToFPosDataQ1[i] * math.cos(-math.radians(((90 * i) / DataPos0) )))
                    self.ToFPosY1.append(ToFPosDataQ1[i] * math.sin(-math.radians(((90 * i) / DataPos0) )))

            for i in range(DataPos2):
                if ToFPosDataQ2[i] != 0xffff:
                    self.ToFPosX2.append(ToFPosDataQ2[i] * math.cos(-math.radians(((90 * i) / DataPos0) + 90)))
                    self.ToFPosY2.append(ToFPosDataQ2[i] * math.sin(-math.radians(((90 * i) / DataPos0) + 90)))

            for i in range(DataPos3):
                if ToFPosDataQ3[i] != 0xffff:
                    self.ToFPosX3.append(ToFPosDataQ3[i] * math.cos(-math.radians(((90 * i) / DataPos0) + 180)))
                    self.ToFPosY3.append(ToFPosDataQ3[i] * math.sin(-math.radians(((90 * i) / DataPos0) + 180)))

            self.totalToFPosX = self.ToFPosX0 + self.ToFPosX1 + self.ToFPosX2 + self.ToFPosX3
            self.totalToFPosY = self.ToFPosY0 + self.ToFPosY1 + self.ToFPosY2 + self.ToFPosY3
            ## plot
            time.sleep(0.001)
            self.update()
            # print(len(ToFPosDataQ0), len(ToFPosDataQ1), len(ToFPosDataQ2), len(ToFPosDataQ3))
            # print(self.ToFPosX0,end='\n')
        except:
            print("show data_error")
            pass


    def Send_Start_Data(self):  ## Start protocol transfer
        global wait_flag , lidar_start, com_start, serial_start
        self.start_btn.setStyleSheet("border-image: url(start2x.png);")
        try:
            wait_flag = 1

            # serial_init = Serial()
            if lidar_start == 1:
                # com_start = 1

                global n
                n = n + 1
                if (n > 255):
                    n = 0

                start_lidar = bytearray()
                start_lidar.append(0xfd)
                start_lidar.append(0xe0)
                start_lidar.append(n)
                start_lidar.append(0x0f)

                CK_A = 0
                CK_B = 0

                for i in range(4):
                    CK_A = CK_A + start_lidar[i]
                    CK_B = CK_A + CK_B

                CK_A = CK_A & 0x000000ff
                CK_B = CK_B & 0x000000ff


                start_lidar.append(CK_A)
                start_lidar.append(CK_B)
                self.ser.write(start_lidar)
                # Serial().ser.write(start_lidar)
                # serial_init.ser.write(start_lidar)
                lidar_start = 0
                self.run_state_txt.setStyleSheet("border-image: url(Running_02.png);")

                print("start")
                # time.sleep(5)
                # create_thread()
            else:
                print("Running")
        except AttributeError:
            print("Check the comfort connection")
        except serial.serialutil.PortNotOpenError:
            print("Make sure Comfort is open")

    def Send_Stop_Data(self):   ## Stop protocol transfer
        global wait_flag, lidar_start, com_start, serial_start

        serial_start = 1
        # com_start = 0
        wait_flag = 0

        # for i in range(2):
        try:
            # serial_init = Serial()

            # print(com_start)
            time.sleep(0.5)

            global n
            self.run_state_txt.setStyleSheet("border-image: url(Stop_02.png);")
            # self.run_state_txt.setText("STOP")
            # self.run_state_txt.setFont(QtGui.QFont("Yu Gothic UI Semibold",20))
            # self.run_state_txt.setStyleSheet("Color : red")

            n = n + 1
            if (n > 255):
                n = 0

            stop_lidar = bytearray()
            stop_lidar.append(0xfd)
            stop_lidar.append(0xe0)
            stop_lidar.append(n)
            stop_lidar.append(0xf0)

            CK_A = 0
            CK_B = 0

            for i in range(4):
                CK_A = CK_A + stop_lidar[i]
                CK_B = CK_A + CK_B

            CK_A = CK_A & 0x000000ff
            CK_B = CK_B & 0x000000ff

            stop_lidar.append(CK_A)
            stop_lidar.append(CK_B)

            self.ser.write(stop_lidar)
            # serial_init.ser.write(stop_lidar)
            # time.sleep(1)
            self.stop_btn.setStyleSheet("border-image: url(stop2x.png);")
            lidar_start = 1
            print("stop")
        except AttributeError:
            print("Check the comfort connection")
        except serial.serialutil.PortNotOpenError:
            self.stop_btn.setStyleSheet("border-image: url(stop2x.png);")
            print("Make sure Comfort is open")




    def Slider_Scale(self):
        self.scale = self.scale_slider.value()
        # self.lidar_graph.setXRange(-(60 * self.scale), (60 * self.scale))
        # self.lidar_graph.setYRange(-(60 * self.scale), (60 * self.scale))
        self.min = -(550+50*self.scale)
        self.max = (550+50*self.scale)
        self.lidar_graph.setXRange(self.min, self.max)
        self.lidar_graph.setYRange(self.min, self.max)
        # self.lidar_graph.setXRange(-(550+50*self.scale), (550+50*self.scale))
        # self.lidar_graph.setYRange(-(550+50*self.scale), (550+50*self.scale))
        # self.range_txt.setText("Current Range (±" + str(self.max) + ")")
        print(550+50*self.scale)
        time.sleep(0.001)

    def Scale(self):
        if self.scale_comboBox.currentText() == "Default":
            self.lidar_graph.setXRange(-1000, 1000)
            self.lidar_graph.setYRange(-1000, 1000)
        elif self.scale_comboBox.currentText() == "500":
            self.lidar_graph.setXRange(-500, 500)
            self.lidar_graph.setYRange(-500, 500)
        elif self.scale_comboBox.currentText() == "1000":
            self.lidar_graph.setXRange(-1000, 1000)
            self.lidar_graph.setYRange(-1000, 1000)
        elif self.scale_comboBox.currentText() == "1500":
            self.lidar_graph.setXRange(-1500, 1500)
            self.lidar_graph.setYRange(-1500, 1500)
        elif self.scale_comboBox.currentText() == "2000":
            self.lidar_graph.setXRange(-2000, 2000)
            self.lidar_graph.setYRange(-2000, 2000)
        elif self.scale_comboBox.currentText() == "3000":
            self.lidar_graph.setXRange(-3000, 3000)
            self.lidar_graph.setYRange(-3000, 3000)
        elif self.scale_comboBox.currentText() == "4000":
            self.lidar_graph.setXRange(-4000, 4000)
            self.lidar_graph.setYRange(-4000, 4000)
        elif self.scale_comboBox.currentText() == "5000":
            self.lidar_graph.setXRange(-5000, 5000)
            self.lidar_graph.setYRange(-5000, 5000)

    ########## Exit ############
    def closeEvent(self, QCloseEvent):
        global com_start
        re = QMessageBox.question(self, "Exit Confirmation", "Are you sure you want to quit?",
                                  QMessageBox.Yes | QMessageBox.No)

        if re == QMessageBox.Yes:
            com_start = 0
            time.sleep(1)
            QCloseEvent.accept()
        else:
            QCloseEvent.ignore()



# class Serial():
#     def __init__(self):
#         global COM
#
#         if serial_start == 1:
#             print("Serial Class /// serial comport = " + COM)
#             self.ser = serial.Serial(
#                 port=COM,
#                 baudrate=115200,
#                 # baudrate=460800,
#                 # 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000
#                 bytesize=serial.EIGHTBITS,
#                 parity=serial.PARITY_NONE,
#                 stopbits=serial.STOPBITS_ONE,
#             )


class create_thread():
    def __init__(self):
        global com_start
        if com_start ==1:
            print("create thread start")
            # serial_init = Serial()
            # serial_init.ser.open()

            get = threading.Thread(target=Get_data)
            get.daemon = True
            get.start()
            time.sleep(1)

            parsing=threading.Thread(target=Parsing_data)
            parsing.daemon = True
            parsing.start()



class Get_data():
    def __init__(self):
        # try:
        global write, com_start
        # serial_init = Serial()
        # serial_init = window.ser

        while True:
            # print(com_start)
            if com_start == 1:
                # print("\nthreading get data")

                    # if serial_init.ser.readable():

                if window.ser.readable():
                    # if Serial().ser.readable():

                    # print("readable ..  get data 동작중...")
                    rxData = window.ser.read(1)  # Get a value by 1 byte
                    # print(rxData)
                    Data = binascii.b2a_hex(rxData).decode('ascii')
                    # print(Data)
                    hexStr = ' 0x'.join(re.findall('(.{2})', Data))
                    # print(hexStr)
                    hexStr = '0x' + hexStr
                    # print(hexStr)

                    self.data = hexStr.split(" ")

                    q_box.put(self.data) # Put data in q_box
                    # print("q_box len : ",len(q_box))
                else:
                    print("ser cant readable")


                # time.sleep(0.001)
            elif com_start == 0:
                # time.sleep(1)
                print("Break Get Data")
                break





class Parsing_data():
    def __init__(self):

        global Data_length, com_start
        self.wait_data = []
        self.distance_data = []
        # print("parsing_start")
        while True:
            # print(com_start)
            if com_start == 1:
                # print("parsing_start21")
                if q_box.empty() :
                    pass
                    # print("q_box is empty")
                else:

                    self.header = q_box.get()

                    ################### distance #######################
                    if (wait_flag == 1):

                        # window.run_state_txt.setText("run")
                        # window.run_state_txt.setStyleSheet("Color : green")
                        if (int(self.header[0], 16) == 0xfe):
                            self.id = q_box.get()

                            if (int(self.id[0], 16) == 0x10):

                                self.distance_index = q_box.get()
                                self.system_sate = q_box.get()
                                self.synchronous = q_box.get()
                                self.angle_resolution = q_box.get()
                                self.output_rate = q_box.get()
                                self.length = q_box.get()
                                self.length2 = q_box.get()

                                Data_length = (int(self.length2[0], 16) << 8 | int(self.length[0], 16)) #
                                Pos_flag = int((self.synchronous[0]), 16)
                                angle = int(self.angle_resolution[0], 16) / 10.0

                                self.distance_data = []
                                self.distance_data.append(self.header[0])
                                self.distance_data.append(self.id[0])
                                self.distance_data.append(self.distance_index[0])
                                self.distance_data.append(self.system_sate[0])
                                self.distance_data.append(self.synchronous[0])
                                self.distance_data.append(self.angle_resolution[0])
                                self.distance_data.append(self.output_rate[0])
                                self.distance_data.append(self.length[0])
                                self.distance_data.append(self.length2[0])


                                for i in range((Data_length*2)+2):
                                    self.distance_data.append(q_box.get()[0])

                                # show value
                                window.index_txt.setText(str(int(self.distance_index[0], 16)))
                                window.systemstate_txt.setText(str(int(self.system_sate[0], 16)))
                                window.synchronous_txt.setText(str(int(self.synchronous[0], 16)))
                                window.angle_txt.setText(str(angle) + " °")
                                window.output_txt.setText(str(int(self.output_rate[0], 16) / 10) + " Hz")
                                window.distance_length_txt.setText(str(Data_length))
                                # print("Distance data :",self.distance_data)
                                # print("Distance data len :",(len(self.distance_data)-9-2)/2+2)
                                # print(Data_length)

                            ####
                            # if com_start == 0 :
                            #     print("Parsing Data Break")
                            #     break
                            ####
                            ## checksum
                            try:

                                self.distance_header = self.distance_data[0]
                                self.distance_message_id = self.distance_data[1]

                            # print("Lidar data")

                                if (self.distance_header == '0xfe'):
                                    if (self.Msg_Checksum() == True):
                                        self.distance = []
                                        for i in range(0, Data_length):
                                            # self.distance.append(int(self.distance_data[10 + (i*2)], 16) << 8 | int(self.distance_data[9 + (i*2)], 16))
                                            self.distance.append((int(self.distance_data[10 + (i * 2)], 16) << 8 | int(self.distance_data[9 + (i * 2)], 16)))

                                        # lidar 사분면 별로 나눈다.
                                        self.Copy_data(Pos_flag, Data_length)
                                        # print(self.distance)

                                        # Drawing
                                        if (Pos_flag == 4):
                                            self.draw()
                                        # print("drawing")

                                        # print("Distance :",self.distance)
                                        # print("Distance len",len(self.distance))
                                        # print(len(self.distance))
                                    else:
                                        print("error_distance_checksum")

                                else:
                                    print("Header error")
                            except IndexError:
                                print("index Error")

                # time.sleep(0.01)
                
            elif com_start == 0:
                print("Parsing Break")
                time.sleep(1)
                break
            # time.sleep(0.001)


                # else:
                #     print("not yet")



    def Msg_Checksum_wait(self):
        CK_A = 0
        CK_B = 0

        for i in range(4):
            CK_A = CK_A + int(self.wait_data[i], 16)
            CK_B = CK_A + CK_B

        CK_A = CK_A & 0x000000ff
        CK_B = CK_B & 0x000000ff

        if(CK_A == int(self.wait_data[4], 16) and CK_B == int(self.wait_data[5], 16)):
            return True
        else:
            return False

    def Msg_Checksum(self):
        CK_A = 0
        CK_B = 0

        for i in range((Data_length*2)+9):
            CK_A = CK_A + int(self.distance_data[i], 16)
            CK_B = CK_A + CK_B

        CK_A = CK_A & 0x000000ff
        CK_B = CK_B & 0x000000ff

        if(CK_A == int(self.distance_data[((Data_length*2)+9)], 16) and CK_B == int(self.distance_data[((Data_length*2)+10)], 16)):
            return True
        else:
            print(False)
            return False



    def draw(self):
        global DataPos0
        global DataPos1
        global DataPos2
        global DataPos3

        global ToFPosDataQ0
        global ToFPosDataQ1
        global ToFPosDataQ2
        global ToFPosDataQ3

        print("-------draw-------")
        window.show_data()

        DataPos0 = 0
        DataPos1 = 0
        DataPos2 = 0
        DataPos3 = 0
        ToFPosDataQ0 = []
        ToFPosDataQ1 = []
        ToFPosDataQ2 = []
        ToFPosDataQ3 = []
        time.sleep(0.0001)


    def Copy_data(self, Pos_flag, Data_length):
        global DataPos0
        global DataPos1
        global DataPos2
        global DataPos3

        if Pos_flag == 1:
            for i in range(Data_length):
                ToFPosDataQ0.append(self.distance[i])
            DataPos0 = Data_length
            # print(Pos_flag, DataPos0)

        elif Pos_flag == 2:
            for i in range(Data_length):
                ToFPosDataQ1.append(self.distance[i])
            DataPos1 = Data_length
            # print(Pos_flag, DataPos1)

        elif Pos_flag == 3:
            for i in range(Data_length):
                ToFPosDataQ2.append(self.distance[i])
            DataPos2 = Data_length
            # print(Pos_flag, DataPos2)

        elif Pos_flag == 4:
            for i in range(Data_length):
                ToFPosDataQ3.append(self.distance[i])
            DataPos3 = Data_length
            # print(Pos_flag, DataPos3)
        else:
            pass






if __name__ == "__main__":

    # QApplication : Class that runs the program
    app = QApplication(sys.argv)
    # app.setAttribute(QtCore.Qt.AA_Use96Dpi)   ## Resolution adjustment
    window = Window()

    # Code showing the program screen
    window.show()

    # ser_thr = threading.Thread(target = Serial)
    # ser_thr.start()

    # serial_init = Serial()  ## Start serial communication

    # thr = threading.Thread(target=create_thread)
    # thr.start()


    # get = threading.Thread(target=Get_data)
    # get.start()
    # #
    # parsing = threading.Thread(target=Parsing_data)
    # parsing.start()

    # Code that enters the program into the event loop (which triggers the program)
    app.exec_()
