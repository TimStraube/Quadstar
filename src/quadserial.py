"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import time
import serial
import re
import numpy as np
import multiprocessing
import config

class Quadserial():
    def __init__(self) -> None:
        """Class for handling the serial interface to the quadcopter, which samples and forwards its measurement data as well as sending the joystick state to the quadcopter using uart.
        """
        super(Quadserial, self).__init__()

        self.clearLine = False
        portnumber = 0
        error = 1
        # Erst ACM-Ports testen
        while (error and portnumber < 64):
            portname = f"/dev/ttyACM{portnumber}"
            print(f"Probiere Port: {portname}")
            try:
                self.serialPort = serial.Serial(
                    portname,
                    115200
                )
                print(f"Erfolgreich verbunden mit {portname}")
                error = 0
            except Exception as e:
                portnumber += 1
                print(f"Serial Error bei {portname}: {e}")
        # Falls kein ACM-Port gefunden, USB-Ports testen
        if error:
            portnumber = 0
            while (error and portnumber < 16):
                portname = f"/dev/ttyUSB{portnumber}"
                print(f"Probiere Port: {portname}")
                try:
                    self.serialPort = serial.Serial(
                        portname,
                        115200
                    )
                    print(f"Erfolgreich verbunden mit {portname}")
                    error = 0
                except Exception as e:
                    portnumber += 1
                    print(f"Serial Error bei {portname}: {e}")
        if error:
            print("Kein serieller Port gefunden! Kommunikation deaktiviert.")
            self.serialPort = None

        self.sample_id = [
            "accx", 
            "accy", 
            "accz", 
            "gieren", 
            "nicken", 
            "rollen",
            "attx",
            "atty",
            "attz",
            "cmd1",
            "cmd2",
            "cmd3",
            "cmd4",
            "estimatex",
            "estimatey",
            "estimatez",
            "qw",
            "qx",
            "qy",
            "qz",
            "w_M0",
            "w_M1",
            "w_M2",
            "w_M3"
        ]
        self.sample_current = np.zeros((
            len(self.sample_id), 
            1
        ))
        self.sample_units = [
            "m/s", 
            "m/s", 
            "m/s",
            "°",
            "°",
            "°",
            "°",
            "°",
            "°",
            "",
            "",
            "",
            "",
            "m/s^2",
            "m/s^2",
            "m/s^2",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            ""
        ]

    def startEinmalsampler(self):
        with open(config.serialname, 'rb') as ser:
            bs = ser.read(2048)

            line = repr(bs)

            if config.doclear:
                print("\033c", end='')

            for i in range(len(self.sample_id)):
                pattern = (
                    f"{self.sample_id[i]}:\s*(-?\d+\.\d+)"
                )
                match = re.search(pattern, line)
                if match:
                    sample = match.group(1)
                    print(
                        f"{self.sample_id[i]}: {sample}{self.sample_units[i]}"
                    )
                    self.sample_current[i] = (sample)

            pattern = r'gestimatex:\s*(-?\d+\.\d+)'

            match = re.search(pattern, line)

            if match:
                test = match.group(1)

                print(f'g estimate x: {test}')
            else:
                pass
            
            pattern = r'gestimatey:\s*(-?\d+\.\d+)'

            match = re.search(pattern, line)

            if match:
                test = match.group(1)

                print(f'g estimate y: {test}')
            else:
                pass
            
            pattern = r'gestimatez:\s*(-?\d+\.\d+)'

            match = re.search(pattern, line)

            if match:
                test = match.group(1)

                print(f'g estimate z: {test}')
            else:
                pass

    def startInfinitSampler(self):
        with open(config.serialname, 'rb') as ser:
            while True:
                bs = ser.read(64)

                line = repr(bs)

                if self.clearLine:
                    print("\033c", end='')

                for i in range(len(self.sample_id)):
                    pattern = (
                        f"{self.sample_id[i]}:\s*(-?\d+\.\d+)"
                    )
                    match = re.search(pattern, line)
                    if match:
                        sample = match.group(1)
                        print(
                            f"{self.sample_id[i]}: {sample}{self.sample_units[i]}"
                        )
                        self.sample_current[i] = (sample)

                pattern = r'gestimatex:\s*(-?\d+\.\d+)'

                match = re.search(pattern, line)

                if match:
                    test = match.group(1)

                    print(f'g estimate x: {test}')
                else:
                    pass
                
                pattern = r'gestimatey:\s*(-?\d+\.\d+)'

                match = re.search(pattern, line)

                if match:
                    test = match.group(1)

                    print(f'g estimate y: {test}')
                else:
                    pass
                
                pattern = r'gestimatez:\s*(-?\d+\.\d+)'

                match = re.search(pattern, line)

                if match:
                    test = match.group(1)

                    print(f'g estimate z: {test}')
                else:
                    pass

                time.sleep(0.08)
    
    def send2uart(self, message):
        if self.serialPort is None:
            print("SerialPort nicht initialisiert! Sendevorgang übersprungen.")
            return
        self.serialPort.flushInput()
        self.serialPort.flushOutput()
        # Nur eine einzelne Zahl als String mit Zeilenumbruch senden
        try:
            num = int(message)
        except (ValueError, TypeError):
            try:
                num = int(float(message))
            except (ValueError, TypeError):
                num = 0
        msg = f"{num}\n".encode()
        self.serialPort.write(msg)

    def reset(self):
        pass

if __name__ == '__main__':
    serialSampler = Quadserial()

    prozessSampler = multiprocessing.Process(
        target = serialSampler.startInfinitSampler
    )
    prozessSampler.start()
    
    prozessSampler.join()

