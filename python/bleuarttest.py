# SPDX-FileCopyrightText: 2020 Dan Halbert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
# this code was originally based on the Adafruit "eval() BLE UART" example

import random
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
            if (chr(buffer[1]) == 'A') and (replyidx == PACKET_DATA_ACC_LEN):
                break  

        if uart_service.in_waiting > 0 :
            char = uart_service.read(1)
            if char == b'!':
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
    global new_data
    if new_data:
        x_data.append(datetime.datetime.now())
        y_data.append(z)
        new_data = False
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return line,

x = 0.0
y = 0.0
z = 0.0
new_data = False
##set up plot
x_data, y_data = [], []

figure = pyplot.figure()
line, = pyplot.plot_date(x_data, y_data, '-')

animation = FuncAnimation(figure, update, interval=25)

while True:
    if not uart_connection:
        print("Trying to connect...")
        for adv in ble.start_scan(ProvideServicesAdvertisement):
            if UARTService in adv.services:
                uart_connection = ble.connect(adv)
                print("Connected")
                break
        ble.stop_scan()

    if uart_connection and uart_connection.connected:
        uart_service = uart_connection[UARTService]
        try:
            send_packet("TSB") # start stream
            while uart_connection.connected:

                r = random.randint(0,64).to_bytes(1, byteorder='big')
                g = random.randint(0,64).to_bytes(1, byteorder='big')
                b = random.randint(0,64).to_bytes(1, byteorder='big')

                # while uart_service.in_waiting > 0:
                #     print(uart_service.read(uart_service.in_waiting))
                # time.sleep(1/1000)
                packet = read_packet(1.5)
                if packet:
                    [x,y,z] = struct.unpack('fff',packet[2:14])
                    print(x,y,z)
                    new_data = True

                pyplot.pause(0.005)

        except KeyboardInterrupt:
            send_packet("TSE") # end stream
            break
                