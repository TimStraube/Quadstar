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
        # Iterate 64 ports until the right one is found
        while (error and portnumber < 64):
            try:
                self.serialPort = serial.Serial(
                    f"/dev/ttyACM{portnumber}",
                    115200
                )
                error = 0
            except:
                portnumber += 1
                print("Serial Error.")

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

    def startEndlossampler(self):
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
    
    def send2uart(self, nachricht):
        self.serialPort.flushInput()
        self.serialPort.flushOutput()
        self.serialPort.write(
            nachricht
        )

    def reset(self):
        pass

if __name__ == '__main__':
    serialSampler = Quadserial()

    prozessSampler = multiprocessing.Process(
        target = serialSampler.startEndlossampler
    )
    prozessSampler.start()
    
    prozessSampler.join()

