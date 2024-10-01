import sys
import pygame
import sched
import time
import inspect
import multiprocessing
import matplotlib.patches as patches
import config
from quadserial import Quadserial
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication 
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QPushButton

class Joystick(QWidget):
    active = True
    connected = 0
    found = False
    # Hz
    frequency = 30 
    # raw values or percentages
    norm = True
    mode = {
        "operation", 
        "testing", 
        "namespace",
        "seriell"
    }
    scheduler = sched.scheduler(time.time, time.sleep)

    def __init__(
        self, 
        mode = {"operation", "seriell"}):
        """Verbinden zum und Sampling des Joystickzustandes sowie weiterleiten der Daten an die serielle Schnittstelle
        """

        super(Joystick, self).__init__()

        self.old_axis_thrust = 0
        self.old_axis_x = 0
        self.old_axis_y = 0
        self.regleran_callback = 0
        self.schubnur_callback = 0
        self.mode = mode


    def init_render(self):
        self.setWindowTitle('Joystick')
        self.setWindowFlag(Qt.FramelessWindowHint) 
        self.setStyleSheet("background-color: white;")

        self.layout = QVBoxLayout()
        
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.position = QPushButton("Connect Joystick")
        self.position.setStyleSheet(
            "background-color: white;"
        )
        # self.position.clicked.connect()
        self.layout.addWidget(self.position)

        self.fig = Figure()
        self.axes = self.fig.add_subplot(
            111, 
            projection='3d'
        )
        self.canvas = FigureCanvas(self.fig)
        # self.canvas.set_title(
        #     "Joystick position\nx: 0% y: 0%"
        # )
        # plot_joystick_position.set_xlim([-100, 100])
        # plot_joystick_position.set_ylim([-100, 100])
        # self.plot_joystick_position.set_aspect('equal')
        # plot_joystick_position.axis('off')
        rectangle = patches.Rectangle(
            (-99, -99),
            198, 198,
            fill=False, edgecolor='black'
        )
        # self.plot_joystick_position.plot([0], [0], 'o-', color='black')
        # self.plot_joystick_position.add_patch(rectangle)

        # self.layout.addWidget(self.canvas)

        self.setLayout(self.layout)

        # self.show()
        
    def start(self, namespace = None, event = None):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.connected = 0
            self.found = False
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            if config.joystick2serial:
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

        self.position.setText("Disconnect")

    def stop(self):
        self.scheduler.cancel()
    
    def status(self):
        return self.active, self.connected

    def update(self, namespace = None, event = None):
        pygame.event.pump()
        if self.norm:
            self.axis_x = 100 * self.joystick.get_axis(0)
            self.axis_y = -100 * self.joystick.get_axis(1)
            self.axis_thrust = 100 * (
                0.5 - (self.joystick.get_axis(2) / 2)
            )
            self.schubnur = self.joystick.get_button(24)
            self.regleran = self.joystick.get_button(25)
        else:
            self.axis_x = self.joystick.get_axis(0)
            self.axis_y = self.joystick.get_axis(1)
            self.axis_thrust = self.joystick.get_axis
            self.regleran = 0
            self.schubnur = 0

        if "operation" in self.mode:
            self.render()
        if "testing" in self.mode:
            self.test()
        if "seriell" in self.mode:
            self.writeSerial()
        if "namespace" in self.mode:
            namespace.axis_x = self.axis_x
            namespace.axis_y = self.axis_y
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
        print("x: " + str(self.axis_x))
        print("y: " + str(self.axis_y))
        print("thrust: " + str(self.axis_thrust) + "\n")

    def writeSerial(self):
        if (
            abs(self.old_axis_thrust - self.axis_thrust) > 0.5 or 
            abs(self.old_axis_x - self.axis_x) > 0.5 or 
            abs(self.old_axis_y - self.axis_y) > 0.5 or
            self.regleran != self.regleran_callback or 
            self.schubnur != self.schubnur_callback):

            message = f"{int(self.axis_thrust + 100)}{int(self.axis_x + 500)}{int(self.axis_y + 500)}{self.regleran}{self.schubnur}\n\r".encode()
            print(message)
            self.serialPort.senden(message)
        self.old_axis_x = self.axis_x
        self.old_axis_y = self.axis_y
        self.old_axis_thrust = self.axis_thrust
        self.regleran_callback = self.regleran
        self.schubnur_callback = self.schubnur

if __name__ == '__main__':
    app = QApplication(sys.argv)

    joystick = Joystick()

    process_gui = multiprocessing.Process(
        target = joystick.start
    )
    process_gui.start()

    joystick.init_render()
    
    process_gui.join()

    sys.exit(app.exec())

   