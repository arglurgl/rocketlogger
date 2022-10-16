# SPDX-FileCopyrightText: 2020 Dan Halbert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
# this code was originally based on the Adafruit "eval() BLE UART" example

import random
from time import sleep
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

ble = BLERadio()

uart_connection = None

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
        while uart_connection.connected:

            r = random.randint(0,64).to_bytes(1, byteorder='big')
            g = random.randint(0,64).to_bytes(1, byteorder='big')
            b = random.randint(0,64).to_bytes(1, byteorder='big')

            command = ("!C".encode("utf-8")+r+g+b)
            checksum = (~sum(command) & 0xFF).to_bytes(1, byteorder='big')
            print("Sending")
            uart_service.write(command+checksum)
            print("Waiting...")
            sleep(0.25)