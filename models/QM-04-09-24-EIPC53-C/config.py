## Parameter
orient = "NED"
usePrecession = bool(False)
Ts = 0.001
Tf = 20
Ti = 0

## Quadlive
port = 5001
liveschrittweite = 1

## Quadtest

testschritte = 1000
defaultpid = False

## Quadtrain

# Ordner des Modells
ordnername = "QM-04-09-24-EIPC53-C"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 1000
# Rendern von Trainingsepisoden
render_modell = False 
start_position = [0, 0, 0]

# NN
pi = [2, 2, 2, 2]
vf = [2, 2, 2, 2]

## Model

# Lernrate
lernrate = 1e-6
# [Motor 1, Motor 2, Motor 3, Motor 4, Schrittweite]
aktionsraum = [1000, 1000, 1000, 1000, 1]
# Parallel Umwelten fürs Training
paralleleumwelten = 1
# Batchgröße
batchmenge = 16

## Quadend2end
# Schritte bis das Modell einen neuen Parameter lernt
render_fortschrittsanzeige = True
belohnungsgewichtung = [0.0, 0.7, 0.0, 0.3]

## PID

aktionsinterval = [0, 10]
random_pid = False
aktionen = 6
# self.controller.vel_P_gain[0] = aktion[0]
# self.controller.vel_P_gain[1] = aktion[0]
# self.controller.vel_P_gain[2] = aktion[1]

# self.controller.vel_I_gain[0] = aktion[2]
# self.controller.vel_I_gain[1] = aktion[2]
# self.controller.vel_I_gain[2] = aktion[3]

# self.controller.vel_D_gain[0] = aktion[4]
# self.controller.vel_D_gain[1] = aktion[4]
# self.controller.vel_D_gain[2] = aktion[5]

## Liveevaluation des Trainings
# ?
evaluationsinterval = 2
# ?
evaluationsfrequenz = 10000

## Quadserial
# Aufräumen der Konsole nach jedem neuen Zustand
doclear = False
# Zustand auf der Konsole visualisieren
doprint = False

## Joystick
# Weiterleiten des Joystickzustandes an die serielle Schnittstelle
joystick2serial = True