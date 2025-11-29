"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import pygame
import sched
import time
import multiprocessing
import os
import csv
from util.serial import Quadserial

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
        # remember last printed values to avoid flooding the terminal
        self._last_printed = None
        # button handling
        self.num_buttons = 0
        self.button_states = []
        # Button mapping (user requested):
        # 23 = IDLE, 24 = ACTIVE, 25 = RECOVERY
        self.idle_button_idx = 23
        self.controller_button_idx = 24  # ACTIVE
        self.recovery_button_idx = 25
        self.recovery_active = 0
        # prepare logging for button events
        self._log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        try:
            os.makedirs(self._log_dir, exist_ok=True)
        except Exception:
            self._log_dir = None
        if self._log_dir:
            self._log_file = os.path.join(self._log_dir, 'button_events.csv')
            # create file with header if not exists
            if not os.path.exists(self._log_file):
                try:
                    with open(self._log_file, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['ts', 'button', 'state'])
                except Exception:
                    self._log_file = None
        else:
            self._log_file = None
        
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
            # discover buttons and print them for debugging
            self.num_buttons = self.joystick.get_numbuttons()
            print(f"Joystick initialized: {self.num_buttons} buttons available")
            self.button_states = [0] * self.num_buttons
            for i in range(self.num_buttons):
                try:
                    state = self.joystick.get_button(i)
                except Exception:
                    state = 0
                self.button_states[i] = state
                print(f"Button {i}: {state}")

            # choose mapping: prefer original indices if they exist, otherwise fall back
            if self.num_buttons > max(self.controller_button_idx, self.recovery_button_idx):
                print(f"Using controller_button_idx={self.controller_button_idx}, recovery_button_idx={self.recovery_button_idx}")
            else:
                # fallback mapping
                self.controller_button_idx = 0 if self.num_buttons > 0 else -1
                self.recovery_button_idx = 1 if self.num_buttons > 1 else -1
                print(f"Fallback mapping: controller_button_idx={self.controller_button_idx}, recovery_button_idx={self.recovery_button_idx}")
                
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
                self.roll = int(self.joystick.get_axis(0) * 100)
                self.pitch = int(-self.joystick.get_axis(1) * 100)
                self.axis_thrust = int((0.5 - (self.joystick.get_axis(2) / 2)) * 100)
            except:
                # print("Error: Axis could not be read.")
                self.roll = 0
                self.pitch = 0
                self.axis_thrust = 0
            # read all buttons and detect changes
            try:
                for i in range(self.num_buttons):
                    btn = self.joystick.get_button(i)
                    if btn != self.button_states[i]:
                        # log the change
                        ts = time.time()
                        if btn:
                            print(f"Button {i} pressed")
                        else:
                            print(f"Button {i} released")
                        # write to csv if available
                        if self._log_file:
                            try:
                                with open(self._log_file, 'a', newline='') as f:
                                    writer = csv.writer(f)
                                    writer.writerow([ts, i, int(btn)])
                            except Exception:
                                pass
                        self.button_states[i] = btn
                        # print compact button map
                        try:
                            compact = ''.join(str(int(x)) for x in self.button_states)
                            print(f"Buttons: {compact}")
                        except Exception:
                            pass
                # set flags according to mapped indices
                self.schubnur = self.button_states[self.recovery_button_idx] if (0 <= self.recovery_button_idx < self.num_buttons) else 0
                self.controller_active = self.button_states[self.controller_button_idx] if (0 <= self.controller_button_idx < self.num_buttons) else 0
                self.idle_button = self.button_states[self.idle_button_idx] if (0 <= self.idle_button_idx < self.num_buttons) else 0
            except Exception:
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
        # Also send when the recovery ('schubnur') button changed so we can
        # transmit a special recovery flag immediately.
        if (abs(self.old_axis_thrust - self.axis_thrust) > 0.5
            or self.controller_active != self.controller_active_callback
            or self.schubnur != self.schubnur_callback):
            thrust_value = int(self.axis_thrust)
            roll_value = int(self.roll)
            pitch_value = int(self.pitch)
            # include controller flag as fourth field so receiver can set state
            # controller_flag mapping:
            # 0 = IDLE (button 23), 1 = ACTIVE (button 24), 2 = RECOVERY (button 25)
            # Priority: RECOVERY > ACTIVE > IDLE
            if getattr(self, 'schubnur', 0):
                controller_flag = 2
            elif getattr(self, 'controller_active', 0):
                controller_flag = 1
            elif getattr(self, 'idle_button', 0):
                controller_flag = 0
            else:
                # default to IDLE when no explicit button pressed
                controller_flag = 0
            msg = f"{thrust_value},{roll_value},{pitch_value},{controller_flag}\n"
            print(f"Send: {msg.strip()}")
            self.serialPort.send2uart(msg)
        self.roll_old = self.roll
        self.pitch_old = self.pitch
        self.old_axis_thrust = self.axis_thrust
        self.controller_active_callback = self.controller_active
        self.schubnur_callback = self.schubnur
        # Print only the current values when they changed to avoid flooding.
        current = (int(self.axis_thrust), int(self.roll), int(self.pitch))
        if current != self._last_printed:
            # overwrite the current terminal line (no newline)
            try:
                print(f"\rDEBUG: thrust={current[0]}, roll={current[1]}, pitch={current[2]}", end="", flush=True)
            except Exception:
                # fallback to normal print if terminal doesn't support carriage return
                print(f"DEBUG: thrust={current[0]}, roll={current[1]}, pitch={current[2]}")
            self._last_printed = current

if __name__ == '__main__':
    joystick = Joystick()

    process_joystick = multiprocessing.Process(
        target = joystick.start
    )
    process_joystick.start()

