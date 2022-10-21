# SPDX-FileCopyrightText: 2020 Dan Halbert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
# this code was originally based on the Adafruit "eval() BLE UART" example

import random
import numpy
import struct
import time
import datetime
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation



ble = BLERadio()

uart_connection = None

def send_packet(packet_data: str):
    command = (("!"+packet_data).encode("utf-8"))
    checksum = (~sum(command) & 0xFF).to_bytes(1, byteorder='big')
    uart_service.write(command+checksum)

def read_packet(timeout : float):
    timeout_start = time.time()
    replyidx = 0

    PACKET_DATA_ACC_LEN = 15 

    buffer = bytearray()
    start_found = False

    while (time.time() - timeout_start) < timeout:
    
        if len(buffer)>=2:
            if replyidx == PACKET_DATA_ACC_LEN:
                break  

        if uart_service.in_waiting > 0 :
            char = uart_service.read(1)
            if not start_found and char == b'!':
                start_found = True
                replyidx = 0
            if start_found:
                buffer.extend(char)
                replyidx +=1

    if replyidx == 0:
        print("no data/ timeout")
        return

    if (chr(buffer[0]) != '!'):
        print("doesn't start with '!' packet beginning")
        return

    # check checksum
    xsum = (~sum(buffer[:-1]) & 0xFF)
    checksum = buffer[replyidx-1]

    # Throw an error message if the checksum's don't match
    if (xsum != checksum):
        print("Checksum mismatch in packet : ")
        print(buffer)
        return

    # checksum passed!
    return buffer

def update(frame):
    xline.set_data(t_data, x_data)
    yline.set_data(t_data, y_data)
    zline.set_data(t_data, z_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return xline, yline, zline

def update2(frame):
    hline.set_data(t_data, h_data)
    figure2.gca().relim()
    figure2.gca().autoscale_view()
    return hline

##set up plot
t_data, x_data, y_data, z_data, h_data = [], [], [], [], []

figure = pyplot.figure()
xline, = pyplot.plot_date(t_data, x_data, '-')
yline, = pyplot.plot_date(t_data, y_data, '-')
zline, = pyplot.plot_date(t_data, z_data, '-')
animation = FuncAnimation(figure, update, interval=25)

figure2 = pyplot.figure()
hline, = pyplot.plot_date(t_data, h_data, '-')
animation2 = FuncAnimation(figure2, update2, interval=25)

def get_arrow():
    x = 0
    y = 0
    z = 0
    u = x_data[-1:]
    v = y_data[-1:]
    w = z_data[-1:]
    return x,y,z,u,v,w

figure3 = pyplot.figure()
ax3 = figure3.add_subplot(111, projection='3d')
ax3.set_title("Gravity")
quiver = ax3.quiver(*get_arrow())
ax3.set_xlim(-10, 10)
ax3.set_ylim(-10, 10)
ax3.set_zlim(-10, 10)

def update3(frame):
    global quiver
    quiver.remove()
    quiver = ax3.quiver(*get_arrow())

animation3 = FuncAnimation(figure3, update3, interval=25)




while True:
    if not uart_connection:
        print("Trying to connect...")
        for adv in ble.start_scan(ProvideServicesAdvertisement):
            if UARTService in adv.services:
                uart_connection = ble.connect(adv)
                print("Connected")
                break
        ble.stop_scan()

    data_limit = 50

    if uart_connection and uart_connection.connected:
        uart_service = uart_connection[UARTService]
        try:
            #send_packet("TSB") # start stream
            while uart_connection.connected:

                #poll for data
                send_packet("TSP")
                packet = read_packet(1.5)
                if packet and chr(packet[1]) == "A":
                    [x,y,z] = struct.unpack('fff',packet[2:14])
                    t_data.append(datetime.datetime.now())
                    x_data.append(x)
                    y_data.append(y)
                    z_data.append(z)
                    #print(x,y,z)                    
                
                packet = read_packet(1.5)
                if packet and chr(packet[1]) == "H":
                    [temp,press,height] = struct.unpack('fff',packet[2:14])
                    h_data.append(height)
                    #print(temp,press,height)
                
                for curr_list in[t_data, x_data, y_data, z_data, h_data]:
                    if len(curr_list) > data_limit:
                        del curr_list[:1]
                    

                pyplot.pause(0.010)

        except KeyboardInterrupt:
            #send_packet("TSE") # end stream
            break
                