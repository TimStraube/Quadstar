import sys
import pygame
import sched
import time
import multiprocessing
import matplotlib.patches as patches
import config
from quadserial import Quadserial

class Joystick():
    active = True
    connected = 0
    found = False
    # [Hz]
    frequency = 30 
    # raw values or percentages
    norm = True
    mode = {
        "operation", 
        "testing", 
        "namespace",
        "serial"
    }
    scheduler = sched.scheduler(time.time, time.sleep)

    def __init__(
        self, 
        mode = {"operation", "serial"}):
        """Verbinden zum und Sampling des Joystickzustandes sowie weiterleiten der Daten an die serielle Schnittstelle
        """

        super(Joystick, self).__init__()

        self.old_axis_thrust = 0
        self.roll_old = 0
        self.pitch_old = 0
        self.controller_active_callback = 0
        self.schubnur_callback = 0
        self.mode = mode
        
    def start(self, namespace = None, event = None):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.connected = 0
            self.found = False
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            self.serialPort = Quadserial()
                
            self.joystick_connected = 1
            self.active = True
            if "namespace" in self.mode:
                self.scheduler.enter(
                    1 / 60,
                    1,
                    self.update,
                    (self.scheduler, namespace, event)
                )
                self.scheduler.run()
            else:
                self.scheduler.enter(
                    1 / 60,
                    1,
                    self.update,
                    ()
                )
                self.scheduler.run()

    def stop(self):
        self.scheduler.cancel()
    
    def status(self):
        return self.active, self.connected

    def update(self, namespace = None, event = None):
        pygame.event.pump()
        if self.norm:
            try:
                self.roll = 100 * self.joystick.get_axis(0)
                self.pitch = -100 * self.joystick.get_axis(1)
                self.axis_thrust = 100 * (
                    0.5 - (self.joystick.get_axis(2) / 2)
                )
            except:
                # print("Error: Axis could not be read.")
                self.roll = 0
                self.pitch = 0
                self.axis_thrust = 0
            try:
                self.schubnur = self.joystick.get_button(24)
                self.controller_active = self.joystick.get_button(25)
            except:
                # print("Error: Buttons could not be read.")
                self.schubnur = 0
                self.controller_active = 0
        else:
            self.roll = self.joystick.get_axis(0)
            self.pitch = self.joystick.get_axis(1)
            self.axis_thrust = self.joystick.get_axis
            self.controller_active = 0
            self.schubnur = 0

        if "operation" in self.mode:
            self.render()
        if "testing" in self.mode:
            self.test()
        if "serial" in self.mode:
            self.writeSerial()
        if "namespace" in self.mode:
            namespace.roll = self.roll
            namespace.pitch = self.pitch
            namespace.axis_thrust = self.axis_thrust
            event.set()
            self.scheduler.enter(
                1 / self.frequency, 
                1, 
                self.update, 
                (self.scheduler, namespace, event)
            )
        else:
            self.scheduler.enter(
                1 / self.frequency, 
                1, 
                self.update, 
                ()
            )
    
    def render(self):
        pass

    def test(self):
        print("x: " + str(self.roll))
        print("y: " + str(self.pitch))
        print("thrust: " + str(self.axis_thrust) + "\n")

    def writeSerial(self):
        if (
            abs(self.old_axis_thrust - self.axis_thrust) > 0.5 or 
            abs(self.roll_old - self.roll) > 0.5 or 
            abs(self.pitch_old - self.pitch) > 0.5 or
            self.controller_active != self.controller_active_callback or 
            self.schubnur != self.schubnur_callback):

            message = f"{int(self.axis_thrust + 100)}{int(self.roll + 500)}{int(self.pitch + 500)}{self.controller_active}{self.schubnur}\n\r".encode()
            print("Send: " + str(message))
            self.serialPort.senden(message)
        self.roll_old = self.roll
        self.pitch_old = self.pitch
        self.old_axis_thrust = self.axis_thrust
        self.controller_active_callback = self.controller_active
        self.schubnur_callback = self.schubnur

if __name__ == '__main__':
    joystick = Joystick()

    process_joystick = multiprocessing.Process(
        target = joystick.start
    )
    process_joystick.start()

   